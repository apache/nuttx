/****************************************************************************
 * net/sixlowpan/sixlowpan_hc1.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_compresshdr_hc1
 *
 * Description:
 *   Compress IP/UDP header using HC1 and HC_UDP
 *
 *   This function is called by the 6lowpan code to create a compressed
 *   6lowpan packet in the packetbuf buffer from a full IPv6 packet in the
 *   uip_buf buffer.
 *
 *   If we can compress everything, we use HC1 dispatch, if not we use
 *   IPv6 dispatch.  We can compress everything if:
 *
 *     - IP version is
 *     - Flow label and traffic class are 0
 *     - Both src and dest ip addresses are link local
 *     - Both src and dest interface ID are recoverable from lower layer
 *       header
 *     - Next header is either ICMP, UDP or TCP
 *
 *   Moreover, if next header is UDP, we try to compress it using HC_UDP.
 *   This is feasible is both ports are between F0B0 and F0B0 + 15
 *
 *   Resulting header structure:
 *   - For ICMP, TCP, non compressed UDP\n
 *     HC1 encoding = 11111010 (UDP) 11111110 (TCP) 11111100 (ICMP)
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp | HC1 encoding  | IPv6 Hop limit| L4 hdr + data|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | ...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   - For compressed UDP
 *     HC1 encoding = 11111011, HC_UDP encoding = 11100000
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp| HC1 encoding  |  HC_UDP encod.| IPv6 Hop limit|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | src p.| dst p.| UDP checksum                  | L4 data...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * Input Parameters:
 *   radio   - A reference to a radio network device instance
 *   ipv6    - The IPv6 header followd by TCP, UDP, or ICMPv6 header to be
 *             compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr    - Pointer to frame to be compressed.
 *
 * Returned Value:
 *   On success the indications of the defines COMPRESS_HDR_* are returned.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

int sixlowpan_compresshdr_hc1(FAR struct radio_driver_s *radio,
                              FAR const struct ipv6_hdr_s *ipv6,
                              FAR const struct netdev_varaddr_s *destmac,
                              FAR uint8_t *fptr)
{
  FAR uint8_t *hc1 = fptr + g_frame_hdrlen;
  int ret = COMPRESS_HDR_INLINE;

  /* Check if all the assumptions for full compression are valid */

  if (ipv6->vtc != 0x60 || ipv6->tcf != 0 || ipv6->flow != 0 ||
      !sixlowpan_islinklocal(ipv6->srcipaddr) ||
      !sixlowpan_ismacbased(ipv6->srcipaddr, &radio->r_dev.d_mac.radio) ||
      !sixlowpan_islinklocal(ipv6->destipaddr) ||
      !sixlowpan_ismacbased(ipv6->destipaddr, destmac) ||
      ( 1
#ifdef CONFIG_NET_TCP
        && ipv6->proto != IP_PROTO_TCP
#endif
#ifdef CONFIG_NET_UDP
        && ipv6->proto != IP_PROTO_UDP
#endif
#ifdef CONFIG_NET_ICMPv6
        && ipv6->proto != IP_PROTO_ICMP6
#endif
      ))
    {
      /* IPV6 DISPATCH
       * Something cannot be compressed, use IPV6 DISPATCH, compress
       * nothing, copy IPv6 header into the frame buffer
       */

      nwarn("WARNING: Fall back to IPv6 dispatch\n");

      /* IPv6 dispatch header (1 byte) */

      hc1[SIXLOWPAN_HC1_DISPATCH] = SIXLOWPAN_DISPATCH_IPV6;
      g_frame_hdrlen        += SIXLOWPAN_IPV6_HDR_LEN;

      memcpy(fptr + g_frame_hdrlen, ipv6, IPv6_HDRLEN);
      g_frame_hdrlen        += IPv6_HDRLEN;
      g_uncomp_hdrlen       += IPv6_HDRLEN;
    }
  else
    {
      /* HC1 DISPATCH  maximum compresssion:
       * All fields in the IP header but Hop Limit are elided.  If next
       * header is UDP, we compress UDP header using HC2
       */

      hc1[SIXLOWPAN_HC1_DISPATCH] = SIXLOWPAN_DISPATCH_HC1;
      g_uncomp_hdrlen += IPv6_HDRLEN;
      switch (ipv6->proto)
        {
#ifdef CONFIG_NET_ICMPv6
        case IP_PROTO_ICMP6:
          {
            /* HC1 encoding and ttl */

            hc1[SIXLOWPAN_HC1_ENCODING] = 0xfc;
            hc1[SIXLOWPAN_HC1_TTL]      = ipv6->ttl;
            g_frame_hdrlen             += SIXLOWPAN_HC1_HDR_LEN;
          }
          break;
#endif
#ifdef CONFIG_NET_TCP
        case IP_PROTO_TCP:
          {
            /* HC1 encoding and ttl */

            hc1[SIXLOWPAN_HC1_ENCODING] = 0xfe;
            hc1[SIXLOWPAN_HC1_TTL]      = ipv6->ttl;
            g_frame_hdrlen             += SIXLOWPAN_HC1_HDR_LEN;
          }
          break;
#endif
#ifdef CONFIG_NET_UDP
        case IP_PROTO_UDP:
          {
            FAR struct udp_hdr_s *udp =
              &(((FAR struct ipv6udp_hdr_s *)ipv6)->udp);

            /* Try to compress UDP header (we do only full compression).
             * This is feasible if both src and dest ports are between
             * CONFIG_NET_6LOWPAN_MINPORT and CONFIG_NET_6LOWPAN_MINPORT +
             * 15
             */

            ninfo("local/remote port %04x/%04x\n", udp->srcport, udp->destport);

            if (ntohs(udp->srcport)  >=  CONFIG_NET_6LOWPAN_MINPORT &&
                ntohs(udp->srcport)  <  (CONFIG_NET_6LOWPAN_MINPORT + 16) &&
                ntohs(udp->destport) >=  CONFIG_NET_6LOWPAN_MINPORT &&
                ntohs(udp->destport) <  (CONFIG_NET_6LOWPAN_MINPORT + 16))
              {
                FAR uint8_t *hcudp = fptr + g_frame_hdrlen;

                /* HC1 encoding */

                hcudp[SIXLOWPAN_HC1_HC_UDP_HC1_ENCODING] = 0xfb;

                /* HC_UDP encoding, ttl, src and dest ports, checksum */

                hcudp[SIXLOWPAN_HC1_HC_UDP_UDP_ENCODING] = 0xe0;
                hcudp[SIXLOWPAN_HC1_HC_UDP_TTL]          = ipv6->ttl;
                hcudp[SIXLOWPAN_HC1_HC_UDP_PORTS]        =
                  (uint8_t)((ntohs(udp->srcport) - CONFIG_NET_6LOWPAN_MINPORT) << 4) +
                  (uint8_t)((ntohs(udp->destport) - CONFIG_NET_6LOWPAN_MINPORT));

                memcpy(&hcudp[SIXLOWPAN_HC1_HC_UDP_CHKSUM], &udp->udpchksum, 2);

                g_frame_hdrlen  += SIXLOWPAN_HC1_HC_UDP_HDR_LEN;
                g_uncomp_hdrlen += UDP_HDRLEN;
              }
            else
              {
                /* HC1 encoding and ttl */

                hc1[SIXLOWPAN_HC1_ENCODING] = 0xfa;
                hc1[SIXLOWPAN_HC1_TTL]      = ipv6->ttl;
                g_frame_hdrlen             += SIXLOWPAN_HC1_HDR_LEN;
              }

            ret = COMPRESS_HDR_ELIDED;
          }
          break;
#endif /* CONFIG_NET_UDP */

        default:
          {
            /* Test above assures that this will never happen */

            nerr("ERROR: Unhandled protocol\n");
            DEBUGPANIC();
          }
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc1
 *
 * Description:
 *   Uncompress HC1 (and HC_UDP) headers and put them in sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is
 *   HC1.  It processes the frame in the IOB buffer, uncompresses the
 *   header fields, and copies the result in the packet buffer.  At the
 *   end of the decompression, g_frame_hdrlen and uncompressed_hdr_len
 *   are set to the appropriate values
 *
 * Input Parameters:
 *   metadata - Obfuscated MAC metadata including node addressing
 *              information.
 *   iplen    - Equal to 0 if the packet is not a fragment (IP length is
 *              then inferred from the L2 length), non 0 if the packet is
 *              a 1st fragment.
 *   iob      - Pointer to the IOB containing the received frame.
 *   fptr     - Pointer to frame to be uncompressed.
 *   bptr     - Output goes here.  Normally this is a known offset into
 *              d_buf, may be redirected to a "bitbucket" on the case of
 *              FRAGN frames.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, on failure a negater errno value is
 *   returned.
 *
 ****************************************************************************/

int sixlowpan_uncompresshdr_hc1(FAR struct radio_driver_s *radio,
                                FAR const void *metadata, uint16_t iplen,
                                FAR struct iob_s *iob, FAR uint8_t *fptr,
                                FAR uint8_t *bptr)
{
  FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)bptr;
  FAR uint8_t *hc1 = fptr + g_frame_hdrlen;
  struct netdev_varaddr_s addr;
  int ret;

  ninfo("fptr=%p g_frame_hdrlen=%u\n", fptr, g_frame_hdrlen);

  /* Format the IPv6 header in the device d_buf */
  /* Set version, traffic clase, and flow label.  This assumes that Bit 4 is
   * set in HC1.
   */

  ipv6->vtc  = 0x60;  /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  ipv6->tcf  = 0;     /* Bits 0-3: traffic class (LS), 4-bits: flow label (MS) */
  ipv6->flow = 0;     /* 16-bit flow label (LS) */

  g_uncomp_hdrlen += IPv6_HDRLEN;

  /* len[], proto, and ttl depend on the encoding */

  switch (hc1[SIXLOWPAN_HC1_ENCODING] & 0x06)
    {
#ifdef CONFIG_NET_ICMPv6
    case SIXLOWPAN_HC1_NH_ICMPv6:
      ipv6->proto     = IP_PROTO_ICMP6;
      ipv6->ttl       = hc1[SIXLOWPAN_HC1_TTL];
      g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
      break;
#endif
#ifdef CONFIG_NET_TCP
    case SIXLOWPAN_HC1_NH_TCP:
      ipv6->proto     = IP_PROTO_TCP;
      ipv6->ttl       = hc1[SIXLOWPAN_HC1_TTL];
      g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
      break;
#endif
#ifdef CONFIG_NET_UDP
    case SIXLOWPAN_HC1_NH_UDP:
      {
        FAR struct udp_hdr_s *udp = (FAR struct udp_hdr_s *)(bptr + IPv6_HDRLEN);
        FAR uint8_t *hcudp = fptr + g_frame_hdrlen;

        ipv6->proto = IP_PROTO_UDP;

        /* Check for HC_UDP encoding */

        if ((hcudp[SIXLOWPAN_HC1_HC_UDP_HC1_ENCODING] & 0x01) != 0)
          {
            /* UDP header is compressed with HC_UDP */

            if (hcudp[SIXLOWPAN_HC1_HC_UDP_UDP_ENCODING] !=
                SIXLOWPAN_HC_UDP_ALL_C)
              {
                nwarn("WARNING: sixlowpan (uncompress_hdr), packet not supported");
                return -EOPNOTSUPP;
              }

            /* IP TTL */

            ipv6->ttl = hcudp[SIXLOWPAN_HC1_HC_UDP_TTL];

            /* UDP ports, len, checksum */

            udp->srcport =
              htons(CONFIG_NET_6LOWPAN_MINPORT + (hcudp[SIXLOWPAN_HC1_HC_UDP_PORTS] >> 4));
            udp->destport =
              htons(CONFIG_NET_6LOWPAN_MINPORT + (hcudp[SIXLOWPAN_HC1_HC_UDP_PORTS] & 0x0F));

            ninfo("UDP srcport=%04x destport=%04x\n", udp->srcport, udp->destport);

            memcpy(&udp->udpchksum, &hcudp[SIXLOWPAN_HC1_HC_UDP_CHKSUM], 2);

            g_uncomp_hdrlen += UDP_HDRLEN;
            g_frame_hdrlen  += SIXLOWPAN_HC1_HC_UDP_HDR_LEN;
          }
        else
          {
            g_frame_hdrlen  += SIXLOWPAN_HC1_HDR_LEN;
          }
      }
      break;
#endif /* CONFIG_NET_UDP */

    default:
      return -EPROTONOSUPPORT;
    }

  /* Re-create the link-local, mac-based IP address from src/dest node
   * addresses.
   *
   *   PC:  Prefix compressed (link-local prefix assumed)
   *   IC:  Interface identifier elided (derivable from the corresponding
   *        link-layer address).
   */

  if ((hc1[SIXLOWPAN_HC1_ENCODING] & SIXLOWPAN_HC1_SRCADDR_MASK) ==
      SIXLOWPAN_HC1_SRCADDR_PCIC)
    {
      ret = sixlowpan_extract_srcaddr(radio, metadata, &addr);
      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
        }
      else
        {
          sixlowpan_ipfromaddr(&addr, ipv6->srcipaddr);
        }
    }
  else
    {
      nwarn("HC1 srcipaddr encoding not supported: %02x\n",
            hc1[SIXLOWPAN_HC1_ENCODING]);
    }

  ninfo("srcipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ntohs(ipv6->srcipaddr[0]), ntohs(ipv6->srcipaddr[1]),
        ntohs(ipv6->srcipaddr[2]), ntohs(ipv6->srcipaddr[3]),
        ntohs(ipv6->srcipaddr[4]), ntohs(ipv6->srcipaddr[5]),
        ntohs(ipv6->srcipaddr[6]), ntohs(ipv6->srcipaddr[7]));

  if ((hc1[SIXLOWPAN_HC1_ENCODING] & SIXLOWPAN_HC1_DESTADDR_MASK) ==
      SIXLOWPAN_HC1_DESTADDR_PCIC)
    {
      ret = sixlowpan_extract_srcaddr(radio, metadata, &addr);
      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_extract_srcaddr failed: %d\n", ret);
        }
      else
        {
          sixlowpan_ipfromaddr(&addr, ipv6->destipaddr);
        }
    }
  else
    {
      nwarn("HC1 destipaddr encoding not supported: %02x\n",
            hc1[SIXLOWPAN_HC1_ENCODING]);
    }

  ninfo("destipaddr=%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ntohs(ipv6->destipaddr[0]), ntohs(ipv6->destipaddr[1]),
        ntohs(ipv6->destipaddr[2]), ntohs(ipv6->destipaddr[3]),
        ntohs(ipv6->destipaddr[4]), ntohs(ipv6->destipaddr[5]),
        ntohs(ipv6->destipaddr[6]), ntohs(ipv6->destipaddr[7]));

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
      /* This is a 1st fragment */

      ipv6->len[0] = (iplen - IPv6_HDRLEN) >> 8;
      ipv6->len[1] = (iplen - IPv6_HDRLEN) & 0x00FF;
    }

  ninfo("IPv6 len=%02x:%02x\n", ipv6->len[0], ipv6->len[1]);

#ifdef CONFIG_NET_UDP
  /* Length field in UDP header */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      FAR struct udp_hdr_s *udp = (FAR struct udp_hdr_s *)(bptr + IPv6_HDRLEN);
      memcpy(&udp->udplen, &ipv6->len[0], 2);

      ninfo("IPv6 len=%04x\n", udp->udplen);
    }
#endif

  return OK;
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC1 */
