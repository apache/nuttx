/****************************************************************************
 * net/sixlowpan/sixlowpan_hc06.c
 * 6lowpan HC06 implementation (draft-ietf-6lowpan-hc-06, updated to RFC
 * 6282)
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
 *            Joakim Eriksson <joakime@sics.se>
 *            Joel Hoglund <joel@sics.se>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following c/onditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/* FOR HC-06 COMPLIANCE TODO:
 *
 * -Add compression options to UDP, currently only supports
 *  both ports compressed or both ports elided
 * -Verify TC/FL compression works
 * -Add multicast support for M=1 and DAC=1
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/ip.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Used in the encoding of address uncompress rules */

#define UNCOMPRESS_POSTLEN_SHIFT 0
#define UNCOMPRESS_POSTLEN_MASK (0x0f << UNCOMPRESS_POSTLEN_SHIFT)
#  define UNCOMPRESS_POSTLEN(n) (((n) & UNCOMPRESS_POSTLEN_MASK) >> UNCOMPRESS_POSTLEN_SHIFT)
#define UNCOMPRESS_PREFLEN_SHIFT 4
#define UNCOMPRESS_PREFLEN_MASK (0x0f << UNCOMPRESS_PREFLEN_SHIFT)
#  define UNCOMPRESS_PREFLEN(n) (((n) & UNCOMPRESS_PREFLEN_MASK) >> UNCOMPRESS_PREFLEN_SHIFT)
#define UNCOMPRESS_MACBASED (1 << 8)
#define UNCOMPRESS_ZEROPAD  (1 << 9)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* An address context for IPHC address compression each context can have up
 * to 8 bytes
 */

struct sixlowpan_addrcontext_s
{
  uint8_t used;       /* Possibly use as prefix-length */
  uint8_t number;
  uint8_t prefix[8];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HC06 specific variables **************************************************/
/* Use of global variables simplifies the logic and is safe in the multi-
 * device environment because access is serialized via the network lock.
 *
 * But note that state may NOT be preserved from packet-to-packet.
 */

#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
/* Addresses contexts for IPHC. */

static struct sixlowpan_addrcontext_s
  g_hc06_addrcontexts[CONFIG_NET_6LOWPAN_MAXADDRCONTEXT];
#endif

/* Pointer to the byte where to write next inline field. */

static FAR uint8_t *g_hc06ptr;

/* Constant Data ************************************************************/
/* Uncompression of linklocal
 *
 *   0 -> 16 bytes from packet
 *   1 -> 2 bytes from prefix - 16 bytes from packet
 *   2 -> 2 bytes from prefix - 0000::00ff:fe00:XXXX and 2 bytes from packet
 *   3 -> 2 bytes from prefix - Infer 2 or 8 bytes from MAC address
 *
 *   NOTE: ipaddr=the uncompress function does change 0xf to 0x10
 *   NOTE: 0x00 ipaddr=no-autoconfig ipaddr=unspecified
 */

static const uint16_t g_unc_llconf[] =
{
  0x000f, 0x0028, 0x0022, 0x0120
};

/* Uncompression of ctx-based
 *
 *   0 -> 0 bits from packet [unspecified / reserved]
 *   1 -> 8 bytes from prefix - Bunch of zeroes and 8 bytes from packet
 *   2 -> 8 bytes from prefix - 0000::00ff:fe00:XXXX and 2 bytes from packet
 *   3 -> 8 bytes from prefix - Infer 2 or 8 bytes from MAC address
 */

static const uint16_t g_unc_ctxconf[] =
{
  0x0000, 0x0088, 0x0082, 0x0180
};

/* Uncompression of mx-based
 *
 *   0 -> 0 bits from prefix  / 16 bytes inline
 *   1 -> 2 bytes from prefix / 5 bytes inline: ffxx::00xx:xxxx:xxxx
 *   2 -> 2 bytes from prefix / 3 bytes inline: ffxx::00xx:xxxx
 *   3 -> 2 bytes from prefix / 1 byte inline:  ff02::00xx
 *
 *   All other bits required zero padding.
 */

static const uint16_t g_unc_mxconf[] =
{
  0x020f, 0x0225, 0x0223, 0x0221
};

/* Link local prefix */

static const uint8_t g_llprefix[] =
{
  0xfe, 0x80
};

/* TTL uncompression values */

static const uint8_t g_ttl_values[] =
{
  0, 1, 64, 255
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_addrcontext_bynumber
 *
 * Description:
 *   Find the address context with the given number.
 *
 ****************************************************************************/

static FAR struct sixlowpan_addrcontext_s *
  find_addrcontext_bynumber(uint8_t number)
{
  /* Remove code to avoid warnings and save flash if no address context is
   * used.
   */

#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
  int i;

  for (i = 0; i < CONFIG_NET_6LOWPAN_MAXADDRCONTEXT; i++)
    {
      if ((g_hc06_addrcontexts[i].used == 1) &&
           g_hc06_addrcontexts[i].number == number)
        {
          return &g_hc06_addrcontexts[i];
        }
    }
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0 */

  return NULL;
}

/****************************************************************************
 * Name: find_addrcontext_byprefix
 *
 * Description:
 *   Find the address context corresponding to the prefix ipaddr.
 *
 ****************************************************************************/

static FAR struct sixlowpan_addrcontext_s *
  find_addrcontext_byprefix(FAR const net_ipv6addr_t ipaddr)
{
#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
  int i;

  /* Remove code to avoid warnings and save flash if no address context is used */

  for (i = 0; i < CONFIG_NET_6LOWPAN_MAXADDRCONTEXT; i++)
    {
      if ((g_hc06_addrcontexts[i].used == 1) &&
          net_ipv6addr_prefixcmp(&g_hc06_addrcontexts[i].prefix, ipaddr, 64))
        {
          return &g_hc06_addrcontexts[i];
        }
    }
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0 */

  return NULL;
}

/****************************************************************************
 * Name: compress_ipaddr, compress_tagaddr, and compress_laddr
 *
 * Description:
 *   Uncompress addresses based on a prefix and a postfix with zeroes in
 *   between. If the postfix is zero in length it will use the link address
 *   to configure the IP address (autoconf style).
 *
 *   prefpost takes a byte where the first nibble specify prefix count
 *   and the second postfix count (NOTE: 15/0xf ipaddr=16 bytes copy).
 *
 *   compress_tagaddr() accepts a remote, variable length, taged MAC address;
 *   compress_laddr() accepts a local, fixed length MAC address.
 *   compress_ipaddr() is simply the common logic that does not depend on
 *   the size of the MAC address.
 *
 ****************************************************************************/

static uint8_t compress_ipaddr(FAR const net_ipv6addr_t ipaddr, uint8_t bitpos)
{
  if (SIXLOWPAN_IS_IID_16BIT_COMPRESSABLE(ipaddr))
    {
      /* Compress IID to 16 bits: xxxx:xxxx:xxxx:xxxx:0000:00ff:fe00:XXXX */

#ifdef CONFIG_BIG_ENDIAN
      *g_hc06ptr++ = ipaddr[7] >> 8;        /* Preserve big-endian, network order */
      *g_hc06ptr++ = ipaddr[7] & 0xff;
#else
      *g_hc06ptr++ = ipaddr[7] & 0xff;      /* Preserve big-endian, network order */
      *g_hc06ptr++ = ipaddr[7] >> 8;
#endif

      return 2 << bitpos;       /* 16-bits */
    }
  else
    {
      int i;

      /* Do not compress IID: xxxx:xxxx:xxxx:xxxx:IID:IID:IID:IID */

      for (i = 4; i < 8; i++)
        {
#ifdef CONFIG_BIG_ENDIAN
          *g_hc06ptr++ = ipaddr[i] >> 8;    /* Preserve big-endian, network order */
          *g_hc06ptr++ = ipaddr[i] & 0xff;
#else
          *g_hc06ptr++ = ipaddr[i] & 0xff;  /* Preserve big-endian, network order */
          *g_hc06ptr++ = ipaddr[i] >> 8;
#endif
        }

      return 1 << bitpos;       /* 64-bits */
    }
}

static uint8_t compress_tagaddr(FAR const net_ipv6addr_t ipaddr,
                                FAR const struct netdev_varaddr_s *macaddr,
                                uint8_t bitpos)
{
  uint8_t tag;

#ifdef CONFIG_DEBUG_NET_INFO
  ninfo("Compressing bitpos=%u addrlen=%u\n", bitpos, macaddr->nv_addrlen);
  ninfo("            ipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]), ntohs(ipaddr[3]),
        ntohs(ipaddr[4]), ntohs(ipaddr[5]), ntohs(ipaddr[6]), ntohs(ipaddr[7]));

  switch (macaddr->nv_addrlen)
    {
      case 1:
        ninfo("            addr=%02x\n", macaddr->nv_addr[0]);
        break;

      case 2:
        ninfo("            saddr=%02x:%02x\n",
              macaddr->nv_addr[0], macaddr->nv_addr[1]);
        break;

      case 8:
        ninfo("            eaddr=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
              macaddr->nv_addr[0], macaddr->nv_addr[1],
              macaddr->nv_addr[2], macaddr->nv_addr[3],
              macaddr->nv_addr[4], macaddr->nv_addr[5],
              macaddr->nv_addr[6], macaddr->nv_addr[7]);
        break;

      default:
        nerr("ERROR: Unsupported addrlen %u\n", macaddr->nv_addrlen);
        break;
    }
#endif

  if (sixlowpan_ismacbased(ipaddr, macaddr))
    {
      tag = (3 << bitpos);       /* 0-bits */
    }
  else
    {
      tag = compress_ipaddr(ipaddr, bitpos);
    }

  ninfo("Tag=%02x\n", tag);
  return tag;
}

static uint8_t compress_laddr(FAR const net_ipv6addr_t srcipaddr,
                              FAR const struct netdev_varaddr_s *macaddr,
                              uint8_t bitpos)
{
  uint8_t tag;

#ifdef CONFIG_DEBUG_NET_INFO
  ninfo("Compressing bitpos=%u\n", bitpos);
  ninfo("            srcipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ntohs(srcipaddr[0]), ntohs(srcipaddr[1]), ntohs(srcipaddr[2]),
        ntohs(srcipaddr[3]), ntohs(srcipaddr[4]), ntohs(srcipaddr[5]),
        ntohs(srcipaddr[6]), ntohs(srcipaddr[7]));

  switch (macaddr->nv_addrlen)
    {
      case 1:
        ninfo("            addr=%02x\n", macaddr->nv_addr[0]);
        break;

      case 2:
        ninfo("            saddr=%02x:%02x\n",
              macaddr->nv_addr[0], macaddr->nv_addr[1]);
        break;

      case 8:
        ninfo("            eaddr=%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
              macaddr->nv_addr[0], macaddr->nv_addr[1], macaddr->nv_addr[2],
              macaddr->nv_addr[3], macaddr->nv_addr[4], macaddr->nv_addr[5],
              macaddr->nv_addr[6], macaddr->nv_addr[7]);
        break;

       default:
         ninfo("           Unsupported addrlen %u\n", macaddr->nv_addrlen);
    }
#endif

  if (sixlowpan_ismacbased(srcipaddr, macaddr))
    {
      tag = (3 << bitpos);       /* 0-bits */
    }
  else
    {
      tag = compress_ipaddr(srcipaddr, bitpos);
    }

  ninfo("Tag=%02x\n", tag);
  return tag;
}

/****************************************************************************
 * Name: uncompress_addr
 *
 * Description:
 *   Uncompress addresses based on a prefix and a postfix with zeroes in
 *   between. If the postfix is zero in length it will use the link address
 *   to configure the IP address (autoconf style).
 *
 *   prefpost takes a byte where the first nibble specify prefix count
 *   and the second postfix count (NOTE: 15/0xf ipaddr=16 bytes copy).
 *
 ****************************************************************************/

static void uncompress_addr(FAR const struct netdev_varaddr_s *addr,
                            FAR const uint8_t *prefix, uint16_t prefpost,
                            FAR net_ipv6addr_t ipaddr)
{
  FAR const uint8_t *srcptr;
  bool fullmac      = false;
  bool usemac       = (prefpost & UNCOMPRESS_MACBASED) != 0;
  uint8_t prefcount = UNCOMPRESS_PREFLEN(prefpost);
  uint8_t postcount = UNCOMPRESS_POSTLEN(prefpost);
  int destndx;
  int endndx;
  int i;

  /* The value 16 is encoded as 0xf in the 4 bit-fields. */

  prefcount = prefcount == 15 ? 16 : prefcount;
  postcount = postcount == 15 ? 16 : postcount;

  /* Select the data source */

  srcptr = g_hc06ptr;
  if (usemac)
    {
      /* Select the source the address data */

      srcptr = addr->nv_addr;

      /* If the provided postcount is zero and we are taking data from the
       * MAC address, set postcount to the full address length.
       */

      if (postcount == 0)
        {
          postcount = addr->nv_addrlen;
        }

      /* If we are converting the entire MAC address, then we need to some some
       * special bit operations.
       */

      fullmac = (postcount == addr->nv_addrlen);
    }

  /* Copy any prefix */

  if (prefcount > 0)
    {
      memcpy(ipaddr, prefix, prefcount);
    }

  /* Clear bytes between int prefcount and postcount */

  if (prefcount + postcount < 16)
    {
      FAR uint8_t *destptr = (FAR uint8_t *)&ipaddr[0];

      memset(&destptr[prefcount], 0, 16 - (prefcount + postcount));
    }

  /* Copy the remaining data from the source */

  if (postcount > 0)
    {
      /* If there is space for the ...:00ff:fe00:... and if we were not
       * asked t specifically zero pad the address, then add these magic
       * bits to the decoded address.
       */

      if (postcount <= 2 && prefcount < 11 &&
          (prefpost & UNCOMPRESS_ZEROPAD) == 0)
        {
          /* 16 bit uncompression ipaddr=0000:00ff:fe00:xxxx */

          ipaddr[5] = HTONS(0x00ff);
          ipaddr[6] = HTONS(0xfe00);
        }

      /* Handle the even bytes in the address */
      /* If the postcount is even then take extra care with endian-ness */

      destndx = 8 - (postcount >> 1);
      endndx  = 8 - (postcount & 1);

      for (i = destndx; i < endndx; i++)
        {
#ifdef CONFIG_BIG_ENDIAN
          /* Preserve big-endian, network order */

          ipaddr[i] = (uint16_t)srcptr[0] << 8 | (uint16_t)srcptr[1];
#else
          /* Preserve big-endian, network order */

          ipaddr[i] = (uint16_t)srcptr[1] << 8 | (uint16_t)srcptr[0];
#endif
          srcptr += 2;
        }

      /* Handle any remaining odd byte */

      if ((postcount & 1) != 0)
        {
          ipaddr[7] = (uint16_t)(*srcptr) << 8;
        }

      /* If the was a standard MAC based address then toggle */

      if (fullmac)
        {
          ipaddr[7] ^= 0x0200;
        }

      /* If we took the data from packet, then update the packet pointer */

      if (!usemac)
        {
          g_hc06ptr += postcount;
        }
    }
  else if (prefcount > 0)
    {
      /* No IID based configuration if no prefix and no data ipaddr=unspec */

      nwarn("WARNING: No IID based configuration\n");
    }

  ninfo("Uncompressing %d + %d ipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        prefcount, postcount,
        ntohs(ipaddr[0]), ntohs(ipaddr[1]), ntohs(ipaddr[2]), ntohs(ipaddr[3]),
        ntohs(ipaddr[4]), ntohs(ipaddr[5]), ntohs(ipaddr[6]), ntohs(ipaddr[7]));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_hc06_initialize
 *
 * Description:
 *   sixlowpan_hc06_initialize() is called during OS initialization at power-up
 *   reset.  It is called from the common sixlowpan_initialize() function.
 *   sixlowpan_hc06_initialize() configures HC06 networking data structures.
 *   It is called prior to platform-specific driver initialization so that
 *   the 6LoWPAN networking subsystem is prepared to deal with network
 *   driver initialization actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_hc06_initialize(void)
{
#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0
#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1
  int i;
#endif

  /* Preinitialize any address contexts for better header compression
   * (Saves up to 13 bytes per 6lowpan packet).
   */

  g_hc06_addrcontexts[0].used      = 1;
  g_hc06_addrcontexts[0].number    = 0;

  g_hc06_addrcontexts[0].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_0;
  g_hc06_addrcontexts[0].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_1;
  g_hc06_addrcontexts[0].prefix[2] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_2;
  g_hc06_addrcontexts[0].prefix[3] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_3;
  g_hc06_addrcontexts[0].prefix[4] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_4;
  g_hc06_addrcontexts[0].prefix[5] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_5;
  g_hc06_addrcontexts[0].prefix[6] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_6;
  g_hc06_addrcontexts[0].prefix[7] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_0_7;

#if CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1
  for (i = 1; i < CONFIG_NET_6LOWPAN_MAXADDRCONTEXT; i++)
    {
#ifdef CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREINIT_1
      if (i == 1)
        {
          g_hc06_addrcontexts[1].used      = 1;
          g_hc06_addrcontexts[1].number    = 1;

          g_hc06_addrcontexts[1].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_0;
          g_hc06_addrcontexts[1].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_1;
          g_hc06_addrcontexts[1].prefix[2] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_2;
          g_hc06_addrcontexts[1].prefix[3] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_3;
          g_hc06_addrcontexts[1].prefix[4] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_4;
          g_hc06_addrcontexts[1].prefix[5] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_5;
          g_hc06_addrcontexts[1].prefix[6] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_6;
          g_hc06_addrcontexts[1].prefix[7] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_1_7;
        }
      else
#ifdef CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREINIT_2
      if (i == 2)
        {
          g_hc06_addrcontexts[2].used      = 1;
          g_hc06_addrcontexts[2].number    = 2;

          g_hc06_addrcontexts[2].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_0;
          g_hc06_addrcontexts[2].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_1;
          g_hc06_addrcontexts[2].prefix[2] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_2;
          g_hc06_addrcontexts[2].prefix[3] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_3;
          g_hc06_addrcontexts[2].prefix[4] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_4;
          g_hc06_addrcontexts[2].prefix[5] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_5;
          g_hc06_addrcontexts[2].prefix[6] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_6;
          g_hc06_addrcontexts[2].prefix[7] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_7;
        }
      else
#endif /* SIXLOWPAN_CONF_ADDR_CONTEXT_2 */
        {
          g_hc06_addrcontexts[i].used = 0;
        }
#else
      g_hc06_addrcontexts[i].used = 0;
#endif /* SIXLOWPAN_CONF_ADDR_CONTEXT_1 */
    }
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 1 */
#endif /* CONFIG_NET_6LOWPAN_MAXADDRCONTEXT > 0 */
}

/****************************************************************************
 * Name: sixlowpan_compresshdr_hc06
 *
 * Description:
 *   Compress IP/UDP header
 *
 *   This function is called by the 6lowpan code to create a compressed
 *   6lowpan packet in the frame buffer from a full IPv6 packet.
 *
 *     HC-06:
 *
 *     Originally draft-ietf-6lowpan-hc, version 6:
 *     http://tools.ietf.org/html/draft-ietf-6lowpan-hc-06,
 *
 *   Updated to:
 *
 *     RFC 6282:
 *     https://tools.ietf.org/html/rfc6282
 *
 *   NOTE: sixlowpan_compresshdr_hc06() does not support ISA100_UDP header
 *   compression
 *
 *   For LOWPAN_UDP compression, we either compress both ports or none.
 *   General format with LOWPAN_UDP compression is
 *                      1                   2                   3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |0|1|1|TF |N|HLI|C|S|SAM|M|D|DAM| SCI   | DCI   | comp. IPv6 hdr|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | compressed IPv6 fields .....                                  |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LOWPAN_UDP    | non compressed UDP fields ...                 |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | L4 data ...                                                   |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   NOTE: The address context number 00 is reserved for the link local
 *   prefix.  For unicast addresses, if we cannot compress the prefix, we
 *   neither compress the IID.
 *
 * Input Parameters:
 *   radio   - A reference to a radio network device instance
 *   ipv6    - The IPv6 header to be compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr    - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   On success the indications of the defines COMPRESS_HDR_* are returned.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

int sixlowpan_compresshdr_hc06(FAR struct radio_driver_s *radio,
                               FAR const struct ipv6_hdr_s *ipv6,
                               FAR const struct netdev_varaddr_s *destmac,
                               FAR uint8_t *fptr)
{
  FAR uint8_t *iphc = fptr + g_frame_hdrlen;
  FAR struct sixlowpan_addrcontext_s *addrcontext;
  uint8_t iphc0;
  uint8_t iphc1;
  uint8_t tmp;
  int ret = COMPRESS_HDR_INLINE;

  ninfo("fptr=%p g_frame_hdrlen=%u iphc=%p\n", fptr, g_frame_hdrlen, iphc);

  /* As we copy some bit-length fields, in the IPHC encoding bytes,
   * we sometimes use |=
   * If the field is 0, and the current bit value in memory is 1,
   * this does not work. We therefore reset the IPHC encoding here
   */

  iphc0   = SIXLOWPAN_DISPATCH_IPHC;
  iphc1   = 0;
  iphc[2] = 0;         /* Might not be used - but needs to be cleared */

  /* Point to just after the two IPHC bytes we have committed to */

  g_hc06ptr = iphc + 2;

  /* Address handling needs to be made first since it might cause an extra
   * byte with [ SCI | DCI ]
   */

  /* Check if dest address context exists (for allocating third byte)
   *
   * TODO: fix this so that it remembers the looked up values for avoiding two
   * lookups - or set the lookup values immediately
   */

  if (find_addrcontext_byprefix(ipv6->destipaddr) != NULL ||
      find_addrcontext_byprefix(ipv6->srcipaddr) != NULL)
    {
      /* set address context flag and increase g_hc06ptr */

      ninfo("Decompressing dest or src ipaddr. Setting CID\n");
      iphc1 |= SIXLOWPAN_IPHC_CID;
      g_hc06ptr++;
    }

  /* Traffic class, flow label
   *
   * If flow label is 0, compress it. If traffic class is 0, compress it
   * We have to process both in the same time as the offset of traffic class
   * depends on the presence of version and flow label
   */

  /* hc06 format of tc is ECN | DSCP , original is DSCP | ECN */

  tmp = (ipv6->vtc << 4) | (ipv6->tcf >> 4);
  tmp = ((tmp & 0x03) << 6) | (tmp >> 2);

  if (((ipv6->tcf & 0x0f) == 0) && (ipv6->flow == 0))
    {
      /* Flow label can be compressed */

      iphc0 |= SIXLOWPAN_IPHC_TC_10;
      if (((ipv6->vtc & 0x0f) == 0) && ((ipv6->tcf & 0xf0) == 0))
        {
          /* Compress (elide) all */

          iphc0 |= SIXLOWPAN_IPHC_TC_01;
        }
      else
        {
          /* Compress only the flow label */

          *g_hc06ptr = tmp;
          g_hc06ptr += 1;
        }
    }
  else
    {
      /* Flow label cannot be compressed */

      if (((ipv6->vtc & 0x0f) == 0) && ((ipv6->tcf & 0xf0) == 0))
        {
          /* Compress only traffic class */

          iphc0 |= SIXLOWPAN_IPHC_TC_01;
          *g_hc06ptr = (tmp & 0xc0) | (ipv6->tcf & 0x0f);
          memcpy(g_hc06ptr + 1, &ipv6->flow, 2);
          g_hc06ptr += 3;
        }
      else
        {
          /* Compress nothing */

          memcpy(g_hc06ptr, &ipv6->vtc, 4);

          /* But replace the top byte with the new ECN | DSCP format */

          *g_hc06ptr = tmp;
          g_hc06ptr += 4;
        }
    }

  /* Note that the payload length is always compressed */
  /* Next header. We compress it if UDP */

#ifdef CONFIG_NET_UDP
  if (ipv6->proto == IP_PROTO_UDP)
    {
      iphc0 |= SIXLOWPAN_IPHC_NH;
    }
#endif /* CONFIG_NET_UDP */

  if ((iphc0 & SIXLOWPAN_IPHC_NH) == 0)
    {
      *g_hc06ptr = ipv6->proto;
      g_hc06ptr += 1;
    }

  /* Hop limit
   *
   *   if 1: compress, encoding is 01
   *   if 64: compress, encoding is 10
   *   if 255: compress, encoding is 11
   *   else do not compress
   */

  switch (ipv6->ttl)
    {
    case 1:
      iphc0 |= SIXLOWPAN_IPHC_HLIM_1;
      break;

    case 64:
      iphc0 |= SIXLOWPAN_IPHC_HLIM_64;
      break;

    case 255:
      iphc0 |= SIXLOWPAN_IPHC_HLIM_255;
      break;

    default:
      *g_hc06ptr = ipv6->ttl;
      g_hc06ptr += 1;
      break;
    }

  /* Source address - cannot be multicast */

  if (net_is_addr_unspecified(ipv6->srcipaddr))
    {
      ninfo("Compressing unspecified srcipaddr.  Setting SAC\n");

      iphc1 |= SIXLOWPAN_IPHC_SAC;
      iphc1 |= SIXLOWPAN_IPHC_SAM_128;
    }
  else if ((addrcontext = find_addrcontext_byprefix(ipv6->srcipaddr)) != NULL)
    {
      /* Elide the prefix - indicate by CID and set address context + SAC */

      ninfo("Compressing src with address context. Setting CID and SAC context: %d\n",
            addrcontext->number);

      iphc1   |= SIXLOWPAN_IPHC_CID | SIXLOWPAN_IPHC_SAC;
      iphc[2] |= addrcontext->number << 4;

      /* Compression compare with this nodes address (source) */

      iphc1   |= compress_laddr(ipv6->srcipaddr,
                                &radio->r_dev.d_mac.radio,
                                SIXLOWPAN_IPHC_SAM_BIT);
    }

  /* No address context found for the source address */

  else if (net_is_addr_linklocal(ipv6->srcipaddr) &&
           ipv6->srcipaddr[1] == 0 &&  ipv6->srcipaddr[2] == 0 &&
           ipv6->srcipaddr[3] == 0)
    {
      iphc1   |= compress_laddr(ipv6->srcipaddr,
                                &radio->r_dev.d_mac.radio,
                                SIXLOWPAN_IPHC_SAM_BIT);
    }
  else
    {
      /* Send the full source address ipaddr:  SAC = 0, SAM = 00 */

      ninfo("Uncompressable srcipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
            ntohs(ipv6->srcipaddr[0]), ntohs(ipv6->srcipaddr[1]),
            ntohs(ipv6->srcipaddr[2]), ntohs(ipv6->srcipaddr[3]),
            ntohs(ipv6->srcipaddr[4]), ntohs(ipv6->srcipaddr[5]),
            ntohs(ipv6->srcipaddr[6]), ntohs(ipv6->srcipaddr[7]));

      iphc1 |= SIXLOWPAN_IPHC_SAM_128;   /* 128-bits */
      memcpy(g_hc06ptr, ipv6->srcipaddr, 16);
      g_hc06ptr += 16;
    }

  /* Destination address */

  if (net_is_addr_mcast(ipv6->destipaddr))
    {
      /* Address is multicast, try to compress */

      iphc1 |= SIXLOWPAN_IPHC_M;
      if (SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE8(ipv6->destipaddr))
        {
          iphc1 |= SIXLOWPAN_IPHC_MDAM_8;

          /* Use "last" byte ("last" meaning the LS byte in host order.
           * destipaddr is in big-endian network order).
           */

#ifdef CONFIG_ENDIAN_BIG
          *g_hc06ptr = (ipv6->destipaddr[7] & 0xff);
#else
          *g_hc06ptr = (ipv6->destipaddr[7] >> 8);
#endif
          g_hc06ptr += 1;
        }
      else if (SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE32(ipv6->destipaddr))
        {
          FAR uint8_t *iptr = (FAR uint8_t *)ipv6->destipaddr;

          iphc1 |= SIXLOWPAN_IPHC_MDAM_32;

          /* Second byte + the last three */

          *g_hc06ptr = iptr[1];
          memcpy(g_hc06ptr + 1, &iptr[13], 3);
          g_hc06ptr += 4;
        }
      else if (SIXLOWPAN_IS_MCASTADDR_COMPRESSABLE48(ipv6->destipaddr))
        {
          FAR uint8_t *iptr = (FAR uint8_t *)ipv6->destipaddr;

          iphc1 |= SIXLOWPAN_IPHC_MDAM_48;

          /* Second byte + the last five */

          *g_hc06ptr = iptr[1];
          memcpy(g_hc06ptr + 1, &iptr[11], 5);
          g_hc06ptr += 6;
        }
      else
        {
          iphc1 |= SIXLOWPAN_IPHC_MDAM_128;

          /* Full address */

          memcpy(g_hc06ptr, ipv6->destipaddr, 16);
          g_hc06ptr += 16;
        }
    }
  else
    {
      /* Address is unicast, try to compress */

      if ((addrcontext =  find_addrcontext_byprefix(ipv6->destipaddr)) != NULL)
        {
          /* Elide the prefix */

          iphc1   |= SIXLOWPAN_IPHC_DAC;
          iphc[2] |= addrcontext->number;

          /* Compession compare with link adress (destination) */

          iphc1   |= compress_tagaddr(ipv6->destipaddr, destmac,
                                      SIXLOWPAN_IPHC_DAM_BIT);
        }

      /* No address context found for this address */

      else if (net_is_addr_linklocal(ipv6->destipaddr) &&
               ipv6->destipaddr[1] == 0 && ipv6->destipaddr[2] == 0 &&
               ipv6->destipaddr[3] == 0)
        {
          iphc1 |= compress_tagaddr(ipv6->destipaddr, destmac,
                                    SIXLOWPAN_IPHC_DAM_BIT);
        }

      /* Send the full address */

      else
        {
          iphc1 |= SIXLOWPAN_IPHC_DAM_128;       /* 128-bits */
          memcpy(g_hc06ptr, ipv6->destipaddr, 16);
          g_hc06ptr += 16;
        }
    }

  g_uncomp_hdrlen = IPv6_HDRLEN;

#ifdef CONFIG_NET_UDP
  /* UDP header compression */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      /* The UDP header will follow the IPv6 header */

      FAR struct udp_hdr_s *udp =
        (FAR struct udp_hdr_s *)((FAR uint8_t *)ipv6 + IPv6_HDRLEN);

      ninfo("Uncompressed UDP ports: srcport=%04x destport=%04x\n",
            ntohs(udp->srcport), ntohs(udp->destport));

      /* Mask out the last 4 bits can be used as a mask */

      if (((ntohs(udp->srcport) & 0xfff0) == SIXLOWPAN_UDP_4_BIT_PORT_MIN) &&
          ((ntohs(udp->destport) & 0xfff0) == SIXLOWPAN_UDP_4_BIT_PORT_MIN))
        {
          /* We can compress 12 bits of both source and dest */

          *g_hc06ptr = SIXLOWPAN_NHC_UDP_CS_P_11;

          ninfo("Remove 12b of both source & dest with prefix 0xf0b*\n");

          *(g_hc06ptr + 1) =
            (uint8_t)((ntohs(udp->srcport) - SIXLOWPAN_UDP_4_BIT_PORT_MIN) << 4) +
            (uint8_t)((ntohs(udp->destport) - SIXLOWPAN_UDP_4_BIT_PORT_MIN));

          g_hc06ptr += 2;
        }
      else if ((ntohs(udp->destport) & 0xff00) ==
               SIXLOWPAN_UDP_8_BIT_PORT_MIN)
        {
          /* We can compress 8 bits of dest, leave source. */

          *g_hc06ptr = SIXLOWPAN_NHC_UDP_CS_P_01;

          ninfo("Leave source, remove 8 bits of dest with prefix 0xF0\n");

          memcpy(g_hc06ptr + 1, &udp->srcport, 2);
          *(g_hc06ptr + 3) =
            (uint8_t) ((ntohs(udp->destport) -
                        SIXLOWPAN_UDP_8_BIT_PORT_MIN));
          g_hc06ptr += 4;
        }
      else if ((ntohs(udp->srcport) & 0xff00) ==
               SIXLOWPAN_UDP_8_BIT_PORT_MIN)
        {
          /* We can compress 8 bits of src, leave dest. Copy compressed port */

          *g_hc06ptr = SIXLOWPAN_NHC_UDP_CS_P_10;

          ninfo("Remove 8 bits of source with prefix 0xF0, leave dest. hch: %u\n",
                *g_hc06ptr);

          *(g_hc06ptr + 1) =
            (uint8_t)((ntohs(udp->srcport) - SIXLOWPAN_UDP_8_BIT_PORT_MIN));

          memcpy(g_hc06ptr + 2, &udp->destport, 2);
          g_hc06ptr += 4;
        }
      else
        {
          /* we cannot compress. Copy uncompressed ports, full checksum */

          *g_hc06ptr = SIXLOWPAN_NHC_UDP_CS_P_00;

          nwarn("WARNING: Cannot compress headers\n");

          memcpy(g_hc06ptr + 1, &udp->srcport, 4);
          g_hc06ptr += 5;
        }

      /* Always inline the checksum */

      memcpy(g_hc06ptr, &udp->udpchksum, 2);
      g_hc06ptr       += 2;
      g_uncomp_hdrlen += UDP_HDRLEN;
      ret              = COMPRESS_HDR_ELIDED;
    }
#endif /* CONFIG_NET_UDP */

  /* Before the g_frame_hdrlen operation */

  iphc[0] = iphc0;
  iphc[1] = iphc1;

  g_frame_hdrlen = g_hc06ptr - fptr;

  ninfo("fptr=%p g_frame_hdrlen=%u iphc=%02x:%02x:%02x g_hc06ptr=%p\n",
         fptr, g_frame_hdrlen, iphc[0], iphc[1], iphc[2], g_hc06ptr);

  return ret;
}

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc06
 *
 * Description:
 *   Uncompress HC06 (i.e., IPHC and LOWPAN_UDP) headers and put them in
 *   sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is HC06.
 *   We process the frame in the IOB buffer, uncompress the header fields,
 *   and copy the result into the driver packet buffer.  At the end of the
 *   decompression, g_frame_hdrlen and g_uncompressed_hdrlen are set to the
 *   appropriate values
 *
 * Input Parameters:
 *   radio    - Reference to a radio network driver state instance.
 *   metadata - Obfuscated MAC metadata including node addressing
 *              information.
 *   iplen    - Equal to 0 if the packet is not a fragment (IP length is
 *              then inferred from the L2 length), non 0 if the packet is
 *              a first fragment.
 *   iob      - Pointer to the IOB containing the received frame.
 *   fptr     - Pointer to frame to be compressed.
 *   bptr     - Output goes here.  Normally this is a known offset into
 *              d_buf, may be redirected to a "bitbucket" on the case of
 *              FRAGN frames.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_uncompresshdr_hc06(FAR struct radio_driver_s *radio,
                                  FAR const void *metadata,
                                  uint16_t iplen, FAR struct iob_s *iob,
                                  FAR uint8_t *fptr, FAR uint8_t *bptr)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)bptr;
  struct netdev_varaddr_s addr;
  FAR uint8_t *iphc;
  uint8_t iphc0;
  uint8_t iphc1;
  uint8_t tmp;
  int ret;

  /* iphc points to IPHC.  At least two byte will be used for the encoding. */

  iphc  = fptr + g_frame_hdrlen;
  iphc0 = iphc[0];
  iphc1 = iphc[1];

  /* g_hc96ptr points to just after the 2-byte minimum IPHC */

  g_hc06ptr = iphc + 2;

  ninfo("fptr=%p g_frame_hdrlen=%u iphc=%02x:%02x:%02x g_hc06ptr=%p\n",
         fptr, g_frame_hdrlen, iphc[0], iphc[1], iphc[2], g_hc06ptr);

  /* Another if the CID flag is set */

  if ((iphc1 & SIXLOWPAN_IPHC_CID) != 0)
    {
      ninfo("CID flag set. Increase header by one\n");
      g_hc06ptr++;
    }

  /* Traffic class and flow label */

  if ((iphc0 & SIXLOWPAN_IPHC_TC_10) == 0)
    {
      /* Flow label are carried inline */

      if ((iphc0 & SIXLOWPAN_IPHC_TC_01) == 0)
        {
          /* Traffic class is carried inline */

          memcpy(&ipv6->tcf, g_hc06ptr + 1, 3);
          tmp        = *g_hc06ptr;
          g_hc06ptr += 4;

          /* hc06 format of tc is ECN | DSCP , original is DSCP | ECN */
          /* set version, pick highest DSCP bits and set in vtc */

          ipv6->vtc  = 0x60 | ((tmp >> 2) & 0x0f);

          /* ECN rolled down two steps + lowest DSCP bits at top two bits */

          ipv6->tcf  = ((tmp >> 2) & 0x30) | (tmp << 6) | (ipv6->tcf & 0x0f);
        }
      else
        {
          /* Traffic class is compressed (set version and no TC) */

          ipv6->vtc    = 0x60;

          /* Highest flow label bits + ECN bits */

          ipv6->tcf = (*g_hc06ptr & 0x0f) | ((*g_hc06ptr >> 2) & 0x30);
          memcpy(&ipv6->flow, g_hc06ptr + 1, 2);
          g_hc06ptr  += 3;
        }
    }
  else
    {
      /* Version is always 6! */
      /* Version and flow label are compressed */

      if ((iphc0 & SIXLOWPAN_IPHC_TC_01) == 0)
        {
          /* Traffic class is inline */

          ipv6->vtc   = 0x60 | ((*g_hc06ptr >> 2) & 0x0f);
          ipv6->tcf   = ((*g_hc06ptr << 6) & 0xC0) | ((*g_hc06ptr >> 2) & 0x30);
          ipv6->flow  = 0;
          g_hc06ptr  += 1;
        }
      else
        {
          /* Traffic class is compressed */

          ipv6->vtc   = 0x60;
          ipv6->tcf   = 0;
          ipv6->flow  = 0;
        }
    }

  /* Next Header */

  if ((iphc0 & SIXLOWPAN_IPHC_NH) == 0)
    {
      /* Next header is carried inline */

      ipv6->proto = *g_hc06ptr;
      ninfo("Next header inline: %d\n", ipv6->proto);
      g_hc06ptr += 1;
    }

  /* Hop limit */

  if ((iphc0 & SIXLOWPAN_IPHC_HLIM_MASK) != SIXLOWPAN_IPHC_HLIM_INLINE)
    {
      ipv6->ttl  = g_ttl_values[iphc0 & SIXLOWPAN_IPHC_HLIM_MASK];
    }
  else
    {
      ipv6->ttl  = *g_hc06ptr;
      g_hc06ptr += 1;
    }

  /* Put the source address compression mode SAM in the tmp var */

  tmp = ((iphc1 & SIXLOWPAN_IPHC_SAM_MASK) >> SIXLOWPAN_IPHC_SAM_BIT) & 0x03;

  /* Address context based compression */

  ret = sixlowpan_extract_srcaddr(radio, metadata, &addr);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
      return;
    }

  if ((iphc1 & SIXLOWPAN_IPHC_SAC) != 0)
    {
      FAR struct sixlowpan_addrcontext_s *addrcontext;
      uint8_t sci = (iphc1 & SIXLOWPAN_IPHC_CID) ? iphc[2] >> 4 : 0;

      /* Source address - check address context != NULL only if SAM bits are != 0 */

      if (tmp != 0)
        {
          addrcontext = find_addrcontext_bynumber(sci);
          if (addrcontext == NULL)
            {
              nerr("ERROR: Address context not found\n");
              return;
            }
        }

      /* If tmp == 0 we do not have a address context and therefore no prefix */
      /* REVISIT: Source address may not be the same size as the destination
       * address.
       */

      uncompress_addr(&addr,
                      tmp != 0 ? addrcontext->prefix : NULL,
                      g_unc_ctxconf[tmp], ipv6->srcipaddr);
    }
  else
    {
      /* No compression and link local */
      /* REVISIT: Source address may not be the same size as the destination
       * address.
       */

      uncompress_addr(&addr, g_llprefix, g_unc_llconf[tmp],
                      ipv6->srcipaddr);
    }

  /* Destination address */
  /* Put the destination address compression mode into tmp */

  tmp = ((iphc1 & SIXLOWPAN_IPHC_DAM_MASK) >> SIXLOWPAN_IPHC_DAM_BIT) & 0x03;

  /* Multicast compression */

  ret = sixlowpan_extract_destaddr(radio, metadata, &addr);
  if (ret < 0)
    {
      nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
      return;
    }

  if ((iphc1 & SIXLOWPAN_IPHC_M) != 0)
    {
      /* Address context based multicast compression */

      if ((iphc1 & SIXLOWPAN_IPHC_DAC) != 0)
        {
          /* TODO: implement this */
        }
      else
        {
          /* Non-address context based multicast compression
           *
           *   DAM 00: 128 bits
           *   DAM 01: 48 bits   ffxx::00xx:xxxx:xxxx
           *   DAM 10: 32 bits   ffxx::00xx:xxxx
           *   DAM 11: 8 bits    ff02::00xx
           */

          uint8_t prefix[] = { 0xff, 0x02 };
          if (tmp > 0 && tmp < 3)
            {
              prefix[1] = *g_hc06ptr;
              g_hc06ptr++;
            }

          uncompress_addr(&addr, prefix, g_unc_mxconf[tmp],
                          ipv6->destipaddr);
        }
    }
  else
    {
      /* No multicast */
      /* Context based */

      if ((iphc1 & SIXLOWPAN_IPHC_DAC) != 0)
        {
          FAR struct sixlowpan_addrcontext_s *addrcontext;
          uint8_t dci = ((iphc1 & SIXLOWPAN_IPHC_CID) != 0) ? iphc[2] & 0x0f : 0;

          addrcontext = find_addrcontext_bynumber(dci);

          /* All valid cases below need the address context! */

          if (addrcontext == NULL)
            {
              nerr("ERROR: Address context not found\n");
              return;
            }

          uncompress_addr(&addr, addrcontext->prefix, g_unc_ctxconf[tmp],
                          ipv6->destipaddr);
        }
      else
        {
          /* Not address context based ipaddr=link local M = 0, DAC = 0 - same
           * as SAC.
           */

          uncompress_addr(&addr, g_llprefix, g_unc_llconf[tmp],
                          ipv6->destipaddr);
        }
    }

  g_uncomp_hdrlen += IPv6_HDRLEN;

  /* Next header processing - continued */

  if ((iphc0 & SIXLOWPAN_IPHC_NH) != 0)
    {
      FAR struct udp_hdr_s *udp = (FAR struct udp_hdr_s *)(bptr + IPv6_HDRLEN);

      /* The next header is compressed, NHC is following */

      if ((*g_hc06ptr & SIXLOWPAN_NHC_UDP_MASK) == SIXLOWPAN_NHC_UDP_ID)
        {
          uint8_t checksum_compressed;

          ipv6->proto         = IP_PROTO_UDP;
          checksum_compressed = *g_hc06ptr & SIXLOWPAN_NHC_UDP_CHECKSUMC;

          ninfo("Incoming header value: %i\n", *g_hc06ptr);

          switch (*g_hc06ptr & SIXLOWPAN_NHC_UDP_CS_P_11)
            {
            case SIXLOWPAN_NHC_UDP_CS_P_00:
              /* 1 byte for NHC, 4 byte for ports, 2 bytes chksum */

              memcpy(&udp->srcport, g_hc06ptr + 1, 2);
              memcpy(&udp->destport, g_hc06ptr + 3, 2);

              ninfo("Uncompressed UDP ports (ptr+5): %x, %x\n",
                     htons(udp->srcport), htons(udp->destport));

              g_hc06ptr += 5;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_01:
              /* 1 byte for NHC + source 16bit inline, dest = 0xF0 + 8 bit
               * inline
               */

              ninfo("Decompressing destination\n");

              memcpy(&udp->srcport, g_hc06ptr + 1, 2);
              udp->destport =
                htons(SIXLOWPAN_UDP_8_BIT_PORT_MIN + (*(g_hc06ptr + 3)));

              ninfo("Uncompressed UDP ports (ptr+4): %x, %x\n",
                     htons(udp->srcport), htons(udp->destport));

              g_hc06ptr += 4;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_10:
              /* 1 byte for NHC + source = 0xF0 + 8bit inline, dest = 16 bit
               * inline
               */

              ninfo("Decompressing source\n");

              udp->srcport =
                htons(SIXLOWPAN_UDP_8_BIT_PORT_MIN + (*(g_hc06ptr + 1)));
              memcpy(&udp->destport, g_hc06ptr + 2, 2);

              ninfo("Uncompressed UDP ports (ptr+4): %x, %x\n",
                     htons(udp->srcport), htons(udp->destport));

              g_hc06ptr += 4;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_11:
              /* 1 byte for NHC, 1 byte for ports */

              udp->srcport =
                htons(SIXLOWPAN_UDP_4_BIT_PORT_MIN +
                          (*(g_hc06ptr + 1) >> 4));
              udp->destport =
                htons(SIXLOWPAN_UDP_4_BIT_PORT_MIN +
                          ((*(g_hc06ptr + 1)) & 0x0f));

              ninfo("Uncompressed UDP ports (ptr+2): %x, %x\n",
                     htons(udp->srcport), htons(udp->destport));

              g_hc06ptr += 2;
              break;

            default:
              nerr("ERROR: Error unsupported UDP compression\n");
              return;
            }

          if (!checksum_compressed)
            {
              /* Has_checksum, default */

              memcpy(&udp->udpchksum, g_hc06ptr, 2);
              g_hc06ptr += 2;

              ninfo("Checksum included\n");
            }
          else
            {
              nwarn("WARNING: checksum *NOT* included\n");
            }

          g_uncomp_hdrlen += UDP_HDRLEN;
        }
    }

  g_frame_hdrlen = g_hc06ptr - fptr;

  /* IP length field. */

  if (iplen == 0)
    {
      /* This is not a fragmented packet */

      ipv6->len[0] = 0;
      ipv6->len[1] = iob->io_len - g_frame_hdrlen + g_uncomp_hdrlen -
                     IPv6_HDRLEN;
    }
  else
    {
      /* This is a first fragment */

      ipv6->len[0] = (iplen - IPv6_HDRLEN) >> 8;
      ipv6->len[1] = (iplen - IPv6_HDRLEN) & 0x00ff;
    }

  /* Length field in UDP header */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      FAR struct udp_hdr_s *udp = (FAR struct udp_hdr_s *)(bptr + IPv6_HDRLEN);
      memcpy(&udp->udplen, &ipv6->len[0], 2);
    }
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC06 */
