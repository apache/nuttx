/****************************************************************************
 * net/sixlowpan/sixlowpan_framelist.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file derive from Contiki:
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netdev.h>

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* A single IOB must be big enough to hold a full frame */

#if CONFIG_IOB_BUFSIZE < CONFIG_NET_6LOWPAN_FRAMELEN
#  error IOBs must be large enough to hold full IEEE802.14.5 frame
#endif

/* There must be at least enough IOBs to hold the full MTU.  Probably still
 * won't work unless there are a few more.
 */

#if CONFIG_NET_6LOWPAN_MTU > (CONFIG_IOB_BUFSIZE * CONFIG_IOB_NBUFFERS)
#  error Not enough IOBs to hold one full IEEE802.14.5 packet
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_compress_ipv6hdr
 *
 * Description:
 *   IPv6 dispatch "compression" function.  Packets "Compression" when only
 *   IPv6 dispatch is used
 *
 *   There is no compression in this case, all fields are sent
 *   inline. We just add the IPv6 dispatch byte before the packet.
 *
 *   0               1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   | IPv6 Dsp      | IPv6 header and payload ...
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * Input Parameters:
 *   ieee   - Pointer to IEEE802.15.4 MAC driver structure.
 *   destip - Pointer to the IPv6 header to "compress"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_compress_ipv6hdr(FAR struct ieee802154_driver_s *ieee,
                                       FAR const struct ipv6_hdr_s *destip)
{
  /* Indicate the IPv6 dispatch and length */

  *g_rimeptr       = SIXLOWPAN_DISPATCH_IPV6;
  g_rime_hdrlen   += SIXLOWPAN_IPV6_HDR_LEN;

  /* Copy the IPv6 header and adjust pointers */

  memcpy(g_rimeptr + g_rime_hdrlen, destip, IPv6_HDRLEN);
  g_rime_hdrlen   += IPv6_HDRLEN;
  g_uncomp_hdrlen += IPv6_HDRLEN;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_queue_frames
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  This function is called from
 *   send interrupt logic when a TX poll is received.  It formates the
 *   list of frames to be sent by the IEEE802.15.4 MAC driver.
 *
 *   The payload data is in the caller 's_buf' and is of length 's_len'.
 *   Compressed headers will be added and if necessary the packet is
 *   fragmented. The resulting packet/fragments are put in ieee->i_framelist
 *   and the entire list of frames will be delivered to the 802.15.4 MAC via
 *   ieee->i_framelist.
 *
 * Input Parameters:
 *   ieee    - The IEEE802.15.4 MAC driver instance
 *   ipv6hdr - IPv6 header followed by TCP or UDP header.
 *   buf     - Data to send
 *   len     - Length of data to send
 *   destmac - The IEEE802.15.4 MAC address of the destination
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *   This function is expected to fail if the driver is not an IEEE802.15.4
 *   MAC network driver.  In that case, the UDP/TCP will fall back to normal
 *   IPv4/IPv6 formatting.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int sixlowpan_queue_frames(FAR struct ieee802154_driver_s *ieee,
                           FAR const struct ipv6_hdr_s *destip,
                           FAR const void *buf, size_t len,
                           FAR const struct rimeaddr_s *destmac)
{
  FAR struct iob_s *iob;
  int framer_hdrlen;
  struct rimeaddr_s bcastmac;
  uint16_t outlen = 0;

  /* Initialize global data.  Locking the network guarantees that we have
   * exclusive use of the global values for intermediate calculations.
   */

  FRAME_RESET();
  g_uncomp_hdrlen = 0;
  g_rime_hdrlen   = 0;
  /* REVISIT: Do I need this rimeptr? */
  g_rimeptr       = &ieee->i_dev.d_buf[PACKETBUF_HDR_SIZE];

  /* Reset rime buffer, packet buffer metatadata */

  memset(g_pktattrs, 0, PACKETBUF_NUM_ATTRS * sizeof(uint16_t));
  memset(g_pktaddrs, 0, PACKETBUF_NUM_ADDRS * sizeof(struct rimeaddr_s));

  g_pktattrs[PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS] =
    CONFIG_NET_6LOWPAN_MAX_MACTRANSMITS;

  /* Set stream mode for all TCP packets, except FIN packets. */

  if (destip->proto == IP_PROTO_TCP)
    {
      FAR const struct tcp_hdr_s *tcp =
        &((FAR const struct ipv6tcp_hdr_s *)destip)->tcp;

      if ((tcp->flags & TCP_FIN) == 0 &&
          (tcp->flags & TCP_CTL) != TCP_ACK)
        {
          g_pktattrs[PACKETBUF_ATTR_PACKET_TYPE] = PACKETBUF_ATTR_PACKET_TYPE_STREAM;
        }
      else if ((tcp->flags & TCP_FIN) == TCP_FIN)
        {
          g_pktattrs[PACKETBUF_ATTR_PACKET_TYPE] = PACKETBUF_ATTR_PACKET_TYPE_STREAM_END;
        }
    }

  /* The destination address will be tagged to each outbound packet. If the
   * argument destmac is NULL, we are sending a broadcast packet.
   */

  if (destmac == NULL)
    {
      memset(&bcastmac, 0, sizeof(struct rimeaddr_s));
      destmac = &bcastmac;
    }

  /* Pre-allocate the IOB to hold frame or the first fragment, waiting if
   * necessary.
   */

  iob = iob_alloc(false);
  DEBUGASSERT(iob != NULL);

  /* Initialize the IOB */

  iob->io_flink  = NULL;
  iob->io_len    = 0;
  iob->io_offset = 0;
  iob->io_pktlen = 0;

  ninfo("Sending packet len %d\n", len);

#ifndef CONFIG_NET_6LOWPAN_COMPRESSION_IPv6
  if (len >= CONFIG_NET_6LOWPAN_COMPRESSION_THRESHOLD)
    {
      /* Try to compress the headers */

#if defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC1)
      sixlowpan_compresshdr_hc1(ieee, destip, destmac, iob);
#elif defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC06)
      sixlowpan_compresshdr_hc06(ieee, destip, destmac, iob);
#else
#  error No compression specified
#endif
    }
  else
#endif /* !CONFIG_NET_6LOWPAN_COMPRESSION_IPv6 */
    {
      /* Small.. use IPv6 dispatch (no compression) */

      sixlowpan_compress_ipv6hdr(ieee, destip);
    }

  ninfo("Header of len %d\n", g_rime_hdrlen);

  rimeaddr_copy(&g_pktaddrs[PACKETBUF_ADDR_RECEIVER], destmac);

  /* Pre-calculate frame header length. */

  framer_hdrlen = sixlowpan_hdrlen(ieee, ieee->i_panid);
  if (framer_hdrlen < 0)
    {
      /* Failed to determine the size of the header failed. */

      nerr("ERROR: sixlowpan_hdrlen() failed: %d\n", framer_hdrlen);
      return framer_hdrlen;
    }

  /* Check if we need to fragment the packet into several frames */

  if ((int)len - (int)g_uncomp_hdrlen >
      (int)CONFIG_NET_6LOWPAN_MAXPAYLOAD - framer_hdrlen -
      (int)g_rime_hdrlen)
    {
#if CONFIG_NET_6LOWPAN_FRAG
      /* ieee->i_framelist will hold the generated frames; frames will be
       * added at qtail.
       */

      FAR struct iob_s *qtail;
      int verify;

      /* The outbound IPv6 packet is too large to fit into a single 15.4
       * packet, so we fragment it into multiple packets and send them.
       * The first fragment contains frag1 dispatch, then
       * IPv6/HC1/HC06/HC_UDP dispatchs/headers.
       * The following fragments contain only the fragn dispatch.
       */

      ninfo("Fragmentation sending packet len %d\n", len);

      /* Create 1st Fragment */
      /* Add the frame header using the pre-allocated IOB. */

      verify = sixlowpan_framecreate(ieee, iob, ieee->i_panid);
      DEBUGASSERT(verify == framer_hdrlen);
      UNUSED(verify);

      /* Move HC1/HC06/IPv6 header */

      memmove(g_rimeptr + SIXLOWPAN_FRAG1_HDR_LEN, g_rimeptr, g_rime_hdrlen);

      /* Setup up the fragment header.
       *
       * The fragment header contains three fields:  Datagram size, datagram
       * tag and datagram offset:
       *
       * 1. Datagram size describes the total (un-fragmented) payload.
       * 2. Datagram tag identifies the set of fragments and is used to
       *    match fragments of the same payload.
       * 3. Datagram offset identifies the fragment’s offset within the un-
       *    fragmented payload.
       *
       * The fragment header length is 4 bytes for the first header and 5
       * bytes for all subsequent headers.
       */

      PUTINT16(RIME_FRAG_PTR, RIME_FRAG_DISPATCH_SIZE,
              ((SIXLOWPAN_DISPATCH_FRAG1 << 8) | len));
      PUTINT16(RIME_FRAG_PTR, RIME_FRAG_TAG, ieee->i_dgramtag);
      ieee->i_dgramtag++;

      /* Copy payload and enqueue */

      g_rime_hdrlen += SIXLOWPAN_FRAG1_HDR_LEN;
      g_rime_payloadlen =
        (CONFIG_NET_6LOWPAN_MAXPAYLOAD - framer_hdrlen - g_rime_hdrlen) & 0xf8;

      memcpy(g_rimeptr + g_rime_hdrlen,
             (FAR uint8_t *)destip + g_uncomp_hdrlen, g_rime_payloadlen);
      iob->io_len += g_rime_payloadlen + g_rime_hdrlen;

      /* Set outlen to what we already sent from the IP payload */

      outlen = g_rime_payloadlen + g_uncomp_hdrlen;

      ninfo("First fragment: len %d, tag %d\n",
            g_rime_payloadlen, ieee->i_dgramtag);

      /* Add the first frame to the IOB queue */

      ieee->i_framelist = iob;
      qtail             = iob;

      /* Keep track of the total amount of data queue */

      iob->io_pktlen    = iob->io_len;

      /* Create following fragments */

      g_rime_hdrlen = SIXLOWPAN_FRAGN_HDR_LEN;

      while (outlen < len)
        {
          /* Allocate an IOB to hold the next fragment, waiting if
           * necessary.
           */

          iob = iob_alloc(false);
          DEBUGASSERT(iob != NULL);

          /* Initialize the IOB */

          iob->io_flink  = NULL;
          iob->io_len    = 0;
          iob->io_offset = 0;
          iob->io_pktlen = 0;

          /* Add the frame header */

          verify = sixlowpan_framecreate(ieee, iob, ieee->i_panid);
          DEBUGASSERT(verify == framer_hdrlen);
          UNUSED(verify);

          /* Move HC1/HC06/IPv6 header */

          memmove(g_rimeptr + SIXLOWPAN_FRAGN_HDR_LEN, g_rimeptr, g_rime_hdrlen);

          /* Setup up the fragment header */

          PUTINT16(RIME_FRAG_PTR, RIME_FRAG_DISPATCH_SIZE,
                  ((SIXLOWPAN_DISPATCH_FRAGN << 8) | len));
          PUTINT16(RIME_FRAG_PTR, RIME_FRAG_TAG, ieee->i_dgramtag);
          RIME_FRAG_PTR[RIME_FRAG_OFFSET] = outlen >> 3;

          /* Copy payload and enqueue */

          if (len - outlen < g_rime_payloadlen)
            {
              /* Last fragment */

              g_rime_payloadlen = len - outlen;
            }
          else
            {
              g_rime_payloadlen =
                (CONFIG_NET_6LOWPAN_MAXPAYLOAD - framer_hdrlen - g_rime_hdrlen) & 0xf8;
            }

          memcpy(g_rimeptr + g_rime_hdrlen, (FAR uint8_t *)destip + outlen,
                 g_rime_payloadlen);
          iob->io_len = g_rime_payloadlen + g_rime_hdrlen;

          /* Set outlen to what we already sent from the IP payload */

          outlen += (g_rime_payloadlen + g_uncomp_hdrlen);

          ninfo("sixlowpan output: fragment offset %d, len %d, tag %d\n",
                outlen >> 3, g_rime_payloadlen, ieee->i_dgramtag);

          /* Add the next frame to the tail of the IOB queue */

          qtail->io_flink = iob;

          /* Keep track of the total amount of data queue */

          ieee->i_framelist->io_pktlen += iob->io_len;
        }
#else
      nerr("ERROR: Packet too large: %d\n", len);
      nerr("       Cannot to be sent without fragmentation support\n");
      nerr("       dropping packet\n");

      return -E2BIG;
#endif
    }
  else
    {
      int verify;

      /* The packet does not need to be fragmented just copy the "payload"
       * and send in one frame.
       */

      /* Add the frame header to the prealloated IOB. */

      verify = sixlowpan_framecreate(ieee, iob, ieee->i_panid);
      DEBUGASSERT(verify == framer_hdrlen);
      UNUSED(verify);

      /* Copy the payload and queue */

      memcpy(g_rimeptr + g_rime_hdrlen, (FAR uint8_t *)destip + g_uncomp_hdrlen,
             len - g_uncomp_hdrlen);
      iob->io_len = len - g_uncomp_hdrlen + g_rime_hdrlen;

      /* Add the first frame to the IOB queue */

      ieee->i_framelist = iob;

      /* Keep track of the total amount of data queue */

      iob->io_pktlen    = iob->io_len;
    }

  return OK;
}

#endif /* CONFIG_NET_6LOWPAN */
