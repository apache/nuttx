/****************************************************************************
 * net/sixlowpan/sixlowpan_hc06.c
 * 6lowpan HC06 implementation (draft-ietf-6lowpan-hc-06)
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
 * modification, are permitted provided that the following conditions
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
 * -Add stateless multicast option
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC06


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6BUF(ieee) \
  ((FAR struct ipv6_hdr_s *)&(ieee)->i_dev.d_buf)
#define UDPIPv6BUF(ieee) \
  ((FAR struct udp_hdr_s *)&(ieee)->i_dev.d_buf[IPv6_HDRLEN])

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
 *   1 -> 2 bytes from prefix - bunch of zeroes and 8 from packet
 *   2 -> 2 bytes from prefix - 0000::00ff:fe00:XXXX from packet
 *   3 -> 2 bytes from prefix - infer 8 bytes from lladdr
 *
 *   NOTE: => the uncompress function does change 0xf to 0x10
 *   NOTE: 0x00 => no-autoconfig => unspecified
 */

static const uint8_t g_unc_llconf[] = { 0x0f, 0x28, 0x22, 0x20 };

/* Uncompression of ctx-based
 *
 *   0 -> 0 bits from packet [unspecified / reserved]
 *   1 -> 8 bytes from prefix - bunch of zeroes and 8 from packet
 *   2 -> 8 bytes from prefix - 0000::00ff:fe00:XXXX + 2 from packet
 *   3 -> 8 bytes from prefix - infer 8 bytes from lladdr
 */

static const uint8_t g_unc_ctxconf[] = { 0x00, 0x88, 0x82, 0x80 };

/* Uncompression of ctx-based
 *
 *   0 -> 0 bits from packet
 *   1 -> 2 bytes from prefix - bunch of zeroes 5 from packet
 *   2 -> 2 bytes from prefix - zeroes + 3 from packet
 *   3 -> 2 bytes from prefix - infer 1 bytes from lladdr
 */

static const uint8_t g_unc_mxconf[] = { 0x0f, 0x25, 0x23, 0x21 };

/* Link local prefix */

static const uint8_t g_llprefix[] = { 0xfe, 0x80 };

/* TTL uncompression values */

static const uint8_t g_ttl_values[] = { 0, 1, 64, 255 };


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
 * Name: find_addrcontext_bynumber
 *
 * Description:
 *   Find the address context corresponding to the prefix ipaddr.
 *
 ****************************************************************************/

static FAR struct sixlowpan_addrcontext_s *
  find_addrcontext_byprefix(FAR net_ipv6addr_t *ipaddr)
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
 * Name: uncompress_addr
 *
 * Description:
 *   Uncompress addresses based on a prefix and a postfix with zeroes in
 *   between. If the postfix is zero in length it will use the link address
 *   to configure the IP address (autoconf style).
 *
 *   prefpost takes a byte where the first nibble specify prefix count
 *   and the second postfix count (NOTE: 15/0xf => 16 bytes copy).
 *
 ****************************************************************************/

static void uncompress_addr(FAR net_ipv6addr_t ipaddr, uint8_t const prefix[],
                            uint8_t prefpost, FAR struct rimeaddr_s *macaddr)
{
  uint8_t prefcount = prefpost >> 4;
  uint8_t postcount = prefpost & 0x0f;

  /* Full nibble 15 => 16 */

  prefcount = prefcount == 15 ? 16 : prefcount;
  postcount = postcount == 15 ? 16 : postcount;

  if (prefcount > 0)
    {
      memcpy(ipaddr, prefix, prefcount);
    }

  if (prefcount + postcount < 16)
    {
      FAR uint8_t *iptr = (FAR uint8_t *)&ipaddr[0];

      memset(&iptr[prefcount], 0, 16 - (prefcount + postcount));
    }

  if (postcount > 0)
    {
      FAR uint8_t *iptr = (FAR uint8_t *)&ipaddr[0];

      memcpy(&iptr[16 - postcount], g_hc06ptr, postcount);
      if (postcount == 2 && prefcount < 11)
        {
          /* 16 bits uncompression => 0000:00ff:fe00:XXXX */

          iptr[11] = 0xff;
          iptr[12] = 0xfe;
        }

      g_hc06ptr += postcount;
    }
  else if (prefcount > 0)
    {
      /* No IID based configuration if no prefix and no data => unspec */

      sixlowpan_ipfromrime(macaddr, ipaddr);
    }

  ninfo("Uncompressing %d + %d => %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        prefcount, postcount, ipaddr[0], ipaddr[2], ipaddr[3], ipaddr[5],
        ipaddr[5], ipaddr[6], ipaddr[7]);
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
 *   the 6loWPAN networking subsystem is prepared to deal with network
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
        }
      else
#ifdef CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREINIT_2
      if (i == 2)
        {
          g_hc06_addrcontexts[2].used      = 1;
          g_hc06_addrcontexts[2].number    = 2;

          g_hc06_addrcontexts[2].prefix[0] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_0;
          g_hc06_addrcontexts[2].prefix[1] = CONFIG_NET_6LOWPAN_MAXADDRCONTEXT_PREFIX_2_1;
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
 *   6lowpan packet in the packetbuf buffer from a full IPv6 packet in the
 *   uip_buf buffer.
 *
 *     HC-06 (draft-ietf-6lowpan-hc, version 6)
 *     http://tools.ietf.org/html/draft-ietf-6lowpan-hc-06
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
 *   ieee     - A reference to the IEE802.15.4 network device state
 *   destip   - The IPv6 header to be compressed
 *   destmac  - L2 destination address, needed to compress the IP
 *              destination field
 *   iob      - The IOB into which the compressed header should be saved.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_compresshdr_hc06(FAR struct ieee802154_driver_s *ieee,
                                FAR const struct ipv6_hdr_s *destip,
                                FAR const struct rimeaddr_s *destmac,
                                FAR struct iob_s *iob)
{
  /* REVISIT: To be provided */
}

/****************************************************************************
 * Name: sixlowpan_hc06_initialize
 *
 * Description:
 *   Uncompress HC06 (i.e., IPHC and LOWPAN_UDP) headers and put them in
 *   sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is HC06.
 *   We process the packet in the rime buffer, uncompress the header fields,
 *   and copy the result in the sixlowpan buffer.  At the end of the
 *   decompression, g_rime_hdrlen and g_uncompressed_hdrlen are set to the
 *   appropriate values
 *
 * Input Parmeters:
 *   ieee  - A reference to the IEE802.15.4 network device state
 *   iplen - Equal to 0 if the packet is not a fragment (IP length is then
 *           inferred from the L2 length), non 0 if the packet is a first
 *           fragment.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_uncompresshdr_hc06(FAR struct ieee802154_driver_s *ieee,
                                  uint16_t iplen)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF(ieee);
  FAR uint8_t *iphc = RIME_IPHC_BUF;
  uint8_t iphc0;
  uint8_t iphc1;
  uint8_t tmp;

  /* At least two byte will be used for the encoding */

  g_hc06ptr = g_rimeptr + g_rime_hdrlen + 2;

  iphc0 = iphc[0];
  iphc1 = iphc[1];

  /* Another if the CID flag is set */

  if (iphc1 & SIXLOWPAN_IPHC_CID)
    {
      ninfo("IPHC: CID flag set - increase header with one\n");
      g_hc06ptr++;
    }

  /* Traffic class and flow label */

  if ((iphc0 & SIXLOWPAN_IPHC_FL_C) == 0)
    {
      /* Flow label are carried inline */

      if ((iphc0 & SIXLOWPAN_IPHC_TC_C) == 0)
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

      if ((iphc0 & SIXLOWPAN_IPHC_TC_C) == 0)
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

  if ((iphc0 & SIXLOWPAN_IPHC_NH_C) == 0)
    {
      /* Next header is carried inline */

      ipv6->proto = *g_hc06ptr;
      ninfo("IPHC: next header inline: %d\n", ipv6->proto);
      g_hc06ptr += 1;
    }

  /* Hop limit */

  if ((iphc0 & 0x03) != SIXLOWPAN_IPHC_TTL_I)
    {
      ipv6->ttl   = g_ttl_values[iphc0 & 0x03];
    }
  else
    {
      ipv6->ttl   = *g_hc06ptr;
      g_hc06ptr += 1;
    }

  /* Put the source address compression mode SAM in the tmp var */

  tmp = ((iphc1 & SIXLOWPAN_IPHC_SAM_11) >> SIXLOWPAN_IPHC_SAM_BIT) & 0x03;

  /* Address context based compression */

  if (iphc1 & SIXLOWPAN_IPHC_SAC)
    {
      FAR struct sixlowpan_addrcontext_s *addrcontext;
      uint8_t sci = (iphc1 & SIXLOWPAN_IPHC_CID) ? iphc[2] >> 4 : 0;

      /* Source address - check address context != NULL only if SAM bits are != 0 */

      if (tmp != 0)
        {
          addrcontext = find_addrcontext_bynumber(sci);
          if (addrcontext == NULL)
            {
              ninfo("sixlowpan uncompress_hdr: error address context not found\n");
              return;
            }
        }

      /* If tmp == 0 we do not have a Address context and therefore no prefix */

      uncompress_addr(ipv6->srcipaddr,
                      tmp != 0 ? addrcontext->prefix : NULL, g_unc_ctxconf[tmp],
                      (FAR struct rimeaddr_s *)&g_pktaddrs[PACKETBUF_ADDR_SENDER]);
    }
  else
    {
      /* No compression and link local */

      uncompress_addr(ipv6->srcipaddr, g_llprefix, g_unc_llconf[tmp],
                      (FAR struct rimeaddr_s *)&g_pktaddrs[PACKETBUF_ADDR_SENDER]);
    }

  /* Destination address */
  /* put the destination address compression mode into tmp */

  tmp = ((iphc1 & SIXLOWPAN_IPHC_DAM_11) >> SIXLOWPAN_IPHC_DAM_BIT) & 0x03;

  /* Multicast compression */

  if (iphc1 & SIXLOWPAN_IPHC_M)
    {
      /* Address context based multicast compression */

      if (iphc1 & SIXLOWPAN_IPHC_DAC)
        {
          /* TODO: implement this */
        }
      else
        {
          /* non-address context based multicast compression
           *
           *   DAM_00: 128 bits
           *   DAM_01: 48 bits FFXX::00XX:XXXX:XXXX
           *   DAM_10: 32 bits FFXX::00XX:XXXX
           *   DAM_11: 8 bits FF02::00XX
           */

          uint8_t prefix[] = { 0xff, 0x02 };
          if (tmp > 0 && tmp < 3)
            {
              prefix[1] = *g_hc06ptr;
              g_hc06ptr++;
            }

          uncompress_addr(ipv6->destipaddr, prefix, g_unc_mxconf[tmp], NULL);
        }
    }
  else
    {
      /* no multicast */
      /* Context based */

      if (iphc1 & SIXLOWPAN_IPHC_DAC)
        {
          FAR struct sixlowpan_addrcontext_s *addrcontext;
          uint8_t dci    = (iphc1 & SIXLOWPAN_IPHC_CID) ? iphc[2] & 0x0f : 0;

          addrcontext = find_addrcontext_bynumber(dci);

          /* All valid cases below need the address context! */

          if (addrcontext == NULL)
            {
              ninfo("sixlowpan uncompress_hdr: error address context not found\n");
              return;
            }

          uncompress_addr(ipv6->destipaddr, addrcontext->prefix, g_unc_ctxconf[tmp],
                          (FAR struct rimeaddr_s *)&g_pktaddrs[PACKETBUF_ADDR_RECEIVER]);
        }
      else
        {
          /* Not address context based => link local M = 0, DAC = 0 - same as SAC */

          uncompress_addr(ipv6->destipaddr, g_llprefix, g_unc_llconf[tmp],
                          (FAR struct rimeaddr_s *)&g_pktaddrs[PACKETBUF_ADDR_RECEIVER]);
        }
    }

  g_uncomp_hdrlen += IPv6_HDRLEN;

  /* Next header processing - continued */

  if ((iphc0 & SIXLOWPAN_IPHC_NH_C))
    {
      FAR struct udp_hdr_s *udp = UDPIPv6BUF(ieee);

      /* The next header is compressed, NHC is following */

      if ((*g_hc06ptr & SIXLOWPAN_NHC_UDP_MASK) == SIXLOWPAN_NHC_UDP_ID)
        {
          uint8_t checksum_compressed;

          ipv6->proto         = IP_PROTO_UDP;
          checksum_compressed = *g_hc06ptr & SIXLOWPAN_NHC_UDP_CHECKSUMC;

          ninfo("IPHC: Incoming header value: %i\n", *g_hc06ptr);

          switch (*g_hc06ptr & SIXLOWPAN_NHC_UDP_CS_P_11)
            {
            case SIXLOWPAN_NHC_UDP_CS_P_00:
              /* 1 byte for NHC, 4 byte for ports, 2 bytes chksum */

              memcpy(&udp->srcport, g_hc06ptr + 1, 2);
              memcpy(&udp->destport, g_hc06ptr + 3, 2);

              ninfo("IPHC: Uncompressed UDP ports (ptr+5): %x, %x\n",
                     htons(udp->srcport),
                     htons(udp->destport));

              g_hc06ptr += 5;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_01:
              /* 1 byte for NHC + source 16bit inline, dest = 0xF0 + 8 bit
               * inline
               */

              ninfo("IPHC: Decompressing destination\n");

              memcpy(&udp->srcport, g_hc06ptr + 1, 2);
              udp->destport =
                htons(SIXLOWPAN_UDP_8_BIT_PORT_MIN + (*(g_hc06ptr + 3)));

              ninfo("IPHC: Uncompressed UDP ports (ptr+4): %x, %x\n",
                     htons(udp->srcport),
                     htons(udp->destport));

              g_hc06ptr += 4;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_10:
              /* 1 byte for NHC + source = 0xF0 + 8bit inline, dest = 16 bit
               * inline
               */

              ninfo("IPHC: Decompressing source\n");

              udp->srcport =
                htons(SIXLOWPAN_UDP_8_BIT_PORT_MIN + (*(g_hc06ptr + 1)));
              memcpy(&udp->destport, g_hc06ptr + 2, 2);

              ninfo("IPHC: Uncompressed UDP ports (ptr+4): %x, %x\n",
                     htons(udp->srcport),
                     htons(udp->destport));

              g_hc06ptr += 4;
              break;

            case SIXLOWPAN_NHC_UDP_CS_P_11:
              /* 1 byte for NHC, 1 byte for ports */

              udp->srcport =
                htons(SIXLOWPAN_UDP_4_BIT_PORT_MIN +
                          (*(g_hc06ptr + 1) >> 4));
              udp->destport =
                htons(SIXLOWPAN_UDP_4_BIT_PORT_MIN +
                          ((*(g_hc06ptr + 1)) & 0x0F));
              ninfo("IPHC: Uncompressed UDP ports (ptr+2): %x, %x\n",
                     htons(udp->srcport),
                     htons(udp->destport));

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
              ninfo("IPHC: sixlowpan uncompress_hdr: checksum included\n");
            }
          else
            {
              nwarn("WARNING: checksum *NOT* included\n");
            }

          g_uncomp_hdrlen += UDP_HDRLEN;
        }
    }

  g_rime_hdrlen = g_hc06ptr - g_rimeptr;

  /* IP length field. */

  if (iplen == 0)
    {
      /* This is not a fragmented packet */

      ipv6->len[0] = 0;
      ipv6->len[1] = ieee->i_dev.d_len - g_rime_hdrlen + g_uncomp_hdrlen - IPv6_HDRLEN;
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
      FAR struct udp_hdr_s *udp = UDPIPv6BUF(ieee);
      memcpy(&udp->udplen, &ipv6->len[0], 2);
    }
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC06 */
