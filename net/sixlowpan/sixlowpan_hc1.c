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
#include <debug.h>

#include <nuttx/net/netdev.h>
#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN_COMPRESSION_HC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer access helpers */

#define IPv6BUF(dev)    ((FAR struct ipv6_hdr_s *)((dev)->d_buf))
#define UDPIPv6BUF(dev) ((FAR struct udp_hdr_s *)&(dev)->d_buf[IPv6_HDRLEN])

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
 *   This is feasible is both ports are between F0B0 and F0B0 + 15\n\n
 *
 *   Resulting header structure:
 *   - For ICMP, TCP, non compressed UDP\n
 *     HC1 encoding = 11111010 (UDP) 11111110 (TCP) 11111100 (ICMP)\n
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp | HC1 encoding  | IPv6 Hop limit| L4 hdr + data|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | ...
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *   - For compressed UDP
 *     HC1 encoding = 11111011, HC_UDP encoding = 11100000\n
 *                      1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | LoWPAN HC1 Dsp| HC1 encoding  |  HC_UDP encod.| IPv6 Hop limit|
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | src p.| dst p.| UDP checksum                  | L4 data...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * Input Parmeters:
 *   ieee    - A reference to the IEE802.15.4 network device state
 *   ipv6    - The IPv6 header to be compressed
 *   destmac - L2 destination address, needed to compress the IP
 *             destination field
 *   fptr     - Pointer to frame data payload.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_compresshdr_hc1(FAR struct ieee802154_driver_s *ieee,
                               FAR const struct ipv6_hdr_s *ipv6,
                               FAR const struct rimeaddr_s *destmac,
                               FAR uint8_t *fptr)
{
  FAR uint8_t *hc1 = fptr + g_frame_hdrlen;

  /* Check if all the assumptions for full compression are valid */

  if (ipv6->vtc != 0x60 || ipv6->tcf != 0 || ipv6->flow != 0 ||
      !sixlowpan_islinklocal(ipv6->srcipaddr) ||
      !sixlowpan_ismacbased(ipv6->srcipaddr, &ieee->i_nodeaddr) ||
      !sixlowpan_islinklocal(ipv6->destipaddr) ||
      !sixlowpan_ismacbased(ipv6->destipaddr, destmac) ||
      (ipv6->proto != IP_PROTO_ICMP6 && ipv6->proto != IP_PROTO_UDP &&
       ipv6->proto != IP_PROTO_TCP))
    {
      /* IPV6 DISPATCH
       * Something cannot be compressed, use IPV6 DISPATCH,
       * compress nothing, copy IPv6 header in rime buffer
       */

      *fptr             = SIXLOWPAN_DISPATCH_IPV6;
       g_frame_hdrlen  += SIXLOWPAN_IPV6_HDR_LEN;
       memcpy(fptr + g_frame_hdrlen, ipv6, IPv6_HDRLEN);
       g_frame_hdrlen  += IPv6_HDRLEN;
       g_uncomp_hdrlen += IPv6_HDRLEN;
    }
  else
    {
      /* HC1 DISPATCH  maximum compresssion:
       * All fields in the IP header but Hop Limit are elided.  If next
       * header is UDP, we compress UDP header using HC2
       */

      hc1[RIME_HC1_DISPATCH] = SIXLOWPAN_DISPATCH_HC1;
      g_uncomp_hdrlen += IPv6_HDRLEN;
      switch (ipv6->proto)
        {
        case IP_PROTO_ICMP6:
          /* HC1 encoding and ttl */

          hc1[RIME_HC1_ENCODING] = 0xfc;
          hc1[RIME_HC1_TTL] = ipv6->ttl;
          g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
          break;

#if CONFIG_NET_TCP
        case IP_PROTO_TCP:
          /* HC1 encoding and ttl */

          hc1[RIME_HC1_ENCODING] = 0xfe;
          hc1[RIME_HC1_TTL] = ipv6->ttl;
          g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
          break;
#endif /* CONFIG_NET_TCP */

#if CONFIG_NET_UDP
        case IP_PROTO_UDP:
          {
            FAR struct udp_hdr_s *udp = UDPIPv6BUF(&ieee->i_dev);

            /* Try to compress UDP header (we do only full compression).
             * This is feasible if both src and dest ports are between
             * CONFIG_NET_6LOWPAN_MINPORT and CONFIG_NET_6LOWPAN_MINPORT +
             * 15
             */

            ninfo("local/remote port %u/%u\n", udp->srcport, udp->destport);

            if (htons(udp->srcport)  >=  CONFIG_NET_6LOWPAN_MINPORT &&
                htons(udp->srcport)  <  (CONFIG_NET_6LOWPAN_MINPORT + 16) &&
                htons(udp->destport) >=  CONFIG_NET_6LOWPAN_MINPORT &&
                htons(udp->destport) <  (CONFIG_NET_6LOWPAN_MINPORT + 16))
              {
                FAR uint8_t *hcudp = fptr + g_frame_hdrlen;

                /* HC1 encoding */

                hcudp[RIME_HC1_HC_UDP_HC1_ENCODING] = 0xfb;

                /* HC_UDP encoding, ttl, src and dest ports, checksum */

                hcudp[RIME_HC1_HC_UDP_UDP_ENCODING] = 0xe0;
                hcudp[RIME_HC1_HC_UDP_TTL]          = ipv6->ttl;
                hcudp[RIME_HC1_HC_UDP_PORTS]        =
                  (uint8_t)((htons(udp->srcport) - CONFIG_NET_6LOWPAN_MINPORT) << 4) +
                  (uint8_t)((htons(udp->destport) - CONFIG_NET_6LOWPAN_MINPORT));

                memcpy(&hcudp[RIME_HC1_HC_UDP_CHKSUM], &udp->udpchksum, 2);

                g_frame_hdrlen  += SIXLOWPAN_HC1_HC_UDP_HDR_LEN;
                g_uncomp_hdrlen += UDP_HDRLEN;
              }
            else
              {
                /* HC1 encoding and ttl */

                hc1[RIME_HC1_ENCODING] = 0xfa;
                hc1[RIME_HC1_TTL]      = ipv6->ttl;
                g_frame_hdrlen        += SIXLOWPAN_HC1_HDR_LEN;
              }
          }
          break;
#endif /* CONFIG_NET_UDP */
        }
    }
}

/****************************************************************************
 * Name: sixlowpan_uncompresshdr_hc1
 *
 * Description:
 *   Uncompress HC1 (and HC_UDP) headers and put them in sixlowpan_buf
 *
 *   This function is called by the input function when the dispatch is
 *   HC1.  It processes the packet in the rime buffer, uncompresses the
 *   header fields, and copies the result in the sixlowpan buffer.  At the
 *   end of the decompression, g_frame_hdrlen and uncompressed_hdr_len
 *   are set to the appropriate values
 *
 * Input Parameters:
 *   ieee   - A reference to the IEE802.15.4 network device state
 *   iplen  - Equal to 0 if the packet is not a fragment (IP length is then
 *            inferred from the L2 length), non 0 if the packet is a 1st
 *            fragment.
 *   iob    - Pointer to the IOB containing the received frame.
 *   payptr - Pointer to the frame data payload.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, on failure a negater errno value is
 *   returned.
 *
 ****************************************************************************/

int sixlowpan_uncompresshdr_hc1(FAR struct ieee802154_driver_s *ieee,
                                uint16_t iplen, FAR struct iob_s *iob,
                                FAR uint8_t *payptr)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF(&ieee->i_dev);
  FAR uint8_t *hc1 = payptr + g_frame_hdrlen;

  /* Format the IPv6 header in the device d_buf */
  /* Set version, traffic clase, and flow label */

  ipv6->vtc    = 0x60;  /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  ipv6->tcf    = 0;     /* Bits 0-3: traffic class (LS), 4-bits: flow label (MS) */
  ipv6->flow   = 0;     /* 16-bit flow label (LS) */

  /* Use stateless auto-configuration to set source and destination IP
   * addresses.
   */

  sixlowpan_ipfromrime(&g_pktaddrs[PACKETBUF_ADDR_SENDER],
                       ipv6->srcipaddr);
  sixlowpan_ipfromrime(&g_pktaddrs[PACKETBUF_ADDR_RECEIVER],
                       ipv6->destipaddr);
  g_uncomp_hdrlen += IPv6_HDRLEN;

  /* len[], proto, and ttl depend on the encoding */

  switch (hc1[RIME_HC1_ENCODING] & 0x06)
    {
    case SIXLOWPAN_HC1_NH_ICMP6:
      ipv6->proto     = IP_PROTO_ICMP6;
      ipv6->ttl       = hc1[RIME_HC1_TTL];
      g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
      break;

#if CONFIG_NET_TCP
    case SIXLOWPAN_HC1_NH_TCP:
      ipv6->proto     = IP_PROTO_TCP;
      ipv6->ttl       = hc1[RIME_HC1_TTL];
      g_frame_hdrlen += SIXLOWPAN_HC1_HDR_LEN;
      break;
#endif /* CONFIG_NET_TCP */

#if CONFIG_NET_UDP
    case SIXLOWPAN_HC1_NH_UDP:
      {
        FAR struct udp_hdr_s *udp = UDPIPv6BUF(&ieee->i_dev);
        FAR uint8_t *hcudp = payptr + g_frame_hdrlen;

        ipv6->proto = IP_PROTO_UDP;
        if ((hcudp[RIME_HC1_HC_UDP_HC1_ENCODING] & 0x01) != 0)
          {
            /* UDP header is compressed with HC_UDP */

            if (hcudp[RIME_HC1_HC_UDP_UDP_ENCODING] !=
                SIXLOWPAN_HC_UDP_ALL_C)
              {
                nwarn("WARNING: sixlowpan (uncompress_hdr), packet not supported");
                return -EOPNOTSUPP;
              }

            /* IP TTL */

            ipv6->ttl = hcudp[RIME_HC1_HC_UDP_TTL];

            /* UDP ports, len, checksum */

            udp->srcport =
              htons(CONFIG_NET_6LOWPAN_MINPORT + (hcudp[RIME_HC1_HC_UDP_PORTS] >> 4));
            udp->destport =
              htons(CONFIG_NET_6LOWPAN_MINPORT + (hcudp[RIME_HC1_HC_UDP_PORTS] & 0x0F));

            memcpy(&udp->udpchksum, &hcudp[RIME_HC1_HC_UDP_CHKSUM], 2);

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

#if CONFIG_NET_UDP
  /* Length field in UDP header */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      FAR struct udp_hdr_s *udp = UDPIPv6BUF(&ieee->i_dev);
      memcpy(&udp->udplen, &ipv6->len[0], 2);
    }
#endif

  return OK;
}

#endif /* CONFIG_NET_6LOWPAN_COMPRESSION_HC1 */
