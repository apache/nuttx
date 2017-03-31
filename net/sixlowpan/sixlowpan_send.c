/****************************************************************************
 * net/sixlowpan/sixlowpan_send.c
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
#include <errno.h>
#include <debug.h>

#include "nuttx/net/iob.h"
#include "nuttx/net/netdev.h"
#include "nuttx/net/ip.h"
#include "nuttx/net/tcp.h"
#include "nuttx/net/udp.h"
#include "nuttx/net/icmpv6.h"
#include "nuttx/net/sixlowpan.h"

#include "iob/iob.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
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
 * Name: sixlowpan_set_pktattrs
 *
 * Description:
 *   Setup some packet buffer attributes
 *
 * Input Parameters:
 *   ieee - Pointer to IEEE802.15.4 MAC driver structure.
 *   ipv6 - Pointer to the IPv6 header to "compress"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_set_pktattrs(FAR struct ieee802154_driver_s *ieee,
                                   FAR const struct ipv6_hdr_s *ipv6)
{
  int attr = 0;

  /* Set protocol in NETWORK_ID */

  ieee->i_pktattrs[PACKETBUF_ATTR_NETWORK_ID] = ipv6->proto;

  /* Assign values to the channel attribute (port or type + code) */

  if (ipv6->proto == IP_PROTO_UDP)
    {
      FAR struct udp_hdr_s *udp = &((FAR struct ipv6udp_hdr_s *)ipv6)->udp;

      attr = udp->srcport;
      if (udp->destport < attr)
        {
          attr = udp->destport;
        }
    }
  else if (ipv6->proto == IP_PROTO_TCP)
    {
      FAR struct tcp_hdr_s *tcp = &((FAR struct ipv6tcp_hdr_s *)ipv6)->tcp;

      attr = tcp->srcport;
      if (tcp->destport < attr)
        {
          attr = tcp->destport;
        }
    }
  else if (ipv6->proto == IP_PROTO_ICMP6)
    {
      FAR struct icmpv6_iphdr_s *icmp = &((FAR struct ipv6icmp_hdr_s *)ipv6)->icmp;

      attr = icmp->type << 8 | icmp->code;
    }

  ieee->i_pktattrs[PACKETBUF_ATTR_CHANNEL] = attr;
}

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
 *   ieee - Pointer to IEEE802.15.4 MAC driver structure.
 *   ipv6 - Pointer to the IPv6 header to "compress"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sixlowpan_compress_ipv6hdr(FAR struct ieee802154_driver_s *ieee,
                                       FAR const struct ipv6_hdr_s *ipv6)
{
  /* Indicate the IPv6 dispatch and length */

  *ieee->i_rimeptr       = SIXLOWPAN_DISPATCH_IPV6;
  ieee->i_rime_hdrlen   += SIXLOWPAN_IPV6_HDR_LEN;

  /* Copy the IPv6 header and adjust pointers */

  memcpy(ieee->i_rimeptr + ieee->i_rime_hdrlen, ipv6, IPv6_HDRLEN);
  ieee->i_rime_hdrlen   += IPv6_HDRLEN;
  ieee->i_uncomp_hdrlen += IPv6_HDRLEN;
}

/****************************************************************************
 * Name: sixlowpan_send_frame
 *
 * Description:
 *   Send one frame when the IEEE802.15.4 MAC device next polls.
 *
 * Input Parameters:
 *   ieee - Pointer to IEEE802.15.4 MAC driver structure.
 *   iobq - The list of frames to send.
 *
 * Returned Value:
 *   Zero (OK) on success; otherwise a negated errno value is returned.
 *
 ****************************************************************************/

static int sixlowpan_send_frame(FAR struct ieee802154_driver_s *ieee,
                                FAR struct iob_s *iobq)
{
  /* Prepare the frame */
#warning Missing logic
  /* Set up for the TX poll */
  /* When polled, then we need to call sixlowpan_framecreate() to create the
   * frame and copy the payload data into the frame.
   */
#if 0 /* Just some notes of what needs to be done in interrupt handler */
  framer_hdrlen = sixlowpan_createframe(ieee, ieee->i_panid);
  memcpy(ieee->i_rimeptr + ieee->i_rime_hdrlen, (uint8_t *)ipv6 + ieee->i_uncomp_hdrlen, len - ieee->i_uncomp_hdrlen);
  dev->i_framelen = len - ieee->i_uncomp_hdrlen + ieee->i_rime_hdrlen;
#endif
#warning Missing logic
  /* Notify the IEEE802.14.5 MAC driver that we have data to be sent */
#warning Missing logic
  /* Wait for the transfer to complete */
#warning Missing logic
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_send
 *
 * Description:
 *   Process an outgoing UDP or TCP packet.  Takes an IP packet and formats
 *   it to be sent on an 802.15.4 network using 6lowpan.  Called from common
 *   UDP/TCP send logic.
 *
 *  The payload data is in the caller 'buf' and is of length 'len'.
 *  Compressed headers will be added and if necessary the packet is
 *  fragmented. The resulting packet/fragments are put in dev->d_buf and
 *  the first frame will be delivered to the 802.15.4 MAC. via ieee->i_frame.
 *
 * Input Parmeters:
 *
 * Input Parameters:
 *   dev   - The IEEE802.15.4 MAC network driver interface.
 *   ipv6  - IPv6 plus TCP or UDP headers.
 *   buf   - Data to send
 *   len   - Length of data to send
 *   raddr - The IEEE802.15.4 MAC address of the destination
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

int sixlowpan_send(FAR struct net_driver_s *dev,
                   FAR const struct ipv6_hdr_s *ipv6, FAR const void *buf,
                   size_t len, FAR const struct rimeaddr_s *raddr)
{
  FAR struct ieee802154_driver_s *ieee = (FAR struct ieee802154_driver_s *)dev;
  FAR struct iob_s *iob;
  int framer_hdrlen;
  struct rimeaddr_s dest;
  uint16_t outlen = 0;

  /* Initialize device-specific data */

  FRAME_RESET(ieee);
  ieee->i_uncomp_hdrlen = 0;
  ieee->i_rime_hdrlen   = 0;
  /* REVISIT: Do I need this rimeptr? */
  ieee->i_rimeptr = &dev->d_buf[PACKETBUF_HDR_SIZE];

  /* Reset rime buffer, packet buffer metatadata */

  memset(ieee->i_pktattrs, 0, PACKETBUF_NUM_ATTRS * sizeof(uint16_t));
  memset(ieee->i_pktaddrs, 0, PACKETBUF_NUM_ADDRS * sizeof(struct rimeaddr_s));

  ieee->i_pktattrs[PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS] =
    CONFIG_NET_6LOWPAN_MAX_MACTRANSMITS;

#ifdef CONFIG_NET_6LOWPAN_SNIFFER
  if (g_sixlowpan_sniffer != NULL)
    {
      /* Reset rime buffer, packet buffer metatadata */

      memset(ieee->i_pktattrs, 0, PACKETBUF_NUM_ATTRS * sizeof(uint16_t));
      memset(ieee->i_pktaddrs, 0, PACKETBUF_NUM_ADDRS * sizeof(struct rimeaddr_s));

      ieee->i_pktattrs[PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS] =
        CONFIG_NET_6LOWPAN_MAX_MACTRANSMITS;

      /* Call the attribution when the callback comes, but set attributes here */

      sixlowpan_set_pktattrs(ieee, ipv6);
    }
#endif

  /* Reset rime buffer, packet buffer metatadata */

  memset(ieee->i_pktattrs, 0, PACKETBUF_NUM_ATTRS * sizeof(uint16_t));
  memset(ieee->i_pktaddrs, 0, PACKETBUF_NUM_ADDRS * sizeof(struct rimeaddr_s));

  ieee->i_pktattrs[PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS] =
    CONFIG_NET_6LOWPAN_MAX_MACTRANSMITS;

  /* Set stream mode for all TCP packets, except FIN packets. */

  if (ipv6->proto == IP_PROTO_TCP)
    {
      FAR const struct tcp_hdr_s *tcp = &((FAR const struct ipv6tcp_hdr_s *)ipv6)->tcp;

      if ((tcp->flags & TCP_FIN) == 0 &&
          (tcp->flags & TCP_CTL) != TCP_ACK)
        {
          ieee->i_pktattrs[PACKETBUF_ATTR_PACKET_TYPE] = PACKETBUF_ATTR_PACKET_TYPE_STREAM;
        }
      else if ((tcp->flags & TCP_FIN) == TCP_FIN)
        {
          ieee->i_pktattrs[PACKETBUF_ATTR_PACKET_TYPE] = PACKETBUF_ATTR_PACKET_TYPE_STREAM_END;
        }
    }

  /* The destination address will be tagged to each outbound packet. If the
   * argument raddr is NULL, we are sending a broadcast packet.
   */

  if (raddr == NULL)
    {
      memset(&dest, 0, sizeof(struct rimeaddr_s));
    }
  else
    {
      rimeaddr_copy(&dest, (FAR const struct rimeaddr_s *)raddr);
    }

  ninfo("Sending packet len %d\n", len);

#ifndef CONFIG_NET_6LOWPAN_COMPRESSION_IPv6
  if (len >= CONFIG_NET_6LOWPAN_COMPRESSION_THRESHOLD)
    {
      /* Try to compress the headers */

#if defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC1)
      sixlowpan_compresshdr_hc1(dev, &dest);
#elif defined(CONFIG_NET_6LOWPAN_COMPRESSION_HC06)
      sixlowpan_compresshdr_hc06(dev, &dest);
#else
#  error No compression specified
#endif
    }
  else
#endif /* !CONFIG_NET_6LOWPAN_COMPRESSION_IPv6 */
    {
      /* Small.. use IPv6 dispatch (no compression) */

      sixlowpan_compress_ipv6hdr(ieee, ipv6);
    }

  ninfo("Header of len %d\n", ieee->i_rime_hdrlen);

  rimeaddr_copy(&ieee->i_pktaddrs[PACKETBUF_ADDR_RECEIVER], &dest);

  /* Pre-calculate frame header length. */

  framer_hdrlen = sixlowpan_hdrlen(ieee, ieee->i_panid);
  if (framer_hdrlen < 0)
    {
      /* Failed to determine the size of the header failed. */

      nerr("ERROR: sixlowpan_framecreate() failed: %d\n", framer_hdrlen);
      return framer_hdrlen;
    }

  /* Check if we need to fragment the packet into several frames */

  if ((int)len - (int)ieee->i_uncomp_hdrlen >
      (int)CONFIG_NET_6LOWPAN_MAXPAYLOAD - framer_hdrlen -
      (int)ieee->i_rime_hdrlen)
    {
#if CONFIG_NET_6LOWPAN_FRAG
      /* qhead will hold the generated frames; Subsequent frames will be
       * added at qtail.
       */

      FAR struct iob_s *qhead;
      FAR struct iob_s *qtail;

      /* The outbound IPv6 packet is too large to fit into a single 15.4
       * packet, so we fragment it into multiple packets and send them.
       * The first fragment contains frag1 dispatch, then
       * IPv6/HC1/HC06/HC_UDP dispatchs/headers.
       * The following fragments contain only the fragn dispatch.
       */

      ninfo("Fragmentation sending packet len %d\n", len);

      /* Allocate an IOB to hold the first fragment, waiting if necessary. */

      iob = iob_alloc(false);
      DEBUGASSERT(iob != NULL);

      /* Create 1st Fragment */
#  warning Missing logic

      /* Move HC1/HC06/IPv6 header */
#  warning Missing logic

      /* FRAG1 dispatch + header
       * Note that the length is in units of 8 bytes
       */
#  warning Missing logic

      /* Copy payload and send */
#  warning Missing logic

      /* Check TX result. */
#  warning Missing logic

      /* Set outlen to what we already sent from the IP payload */
#  warning Missing logic

      /* Add the first frame to the IOB queue */

      qhead         = iob;
      qtail         = iob;
      iob->io_flink = NULL;

      /* Create following fragments
       * Datagram tag is already in the buffer, we need to set the
       * FRAGN dispatch and for each fragment, the offset
       */
#  warning Missing logic

      while (outlen < len)
        {
          /* Allocate an IOB to hold the next fragment, waiting if
           * necessary.
           */

          iob = iob_alloc(false);
          DEBUGASSERT(iob != NULL);

          /* Copy payload */
#  warning Missing logic

          ninfo("sixlowpan output: fragment offset %d, len %d, tag %d\n",
                outlen >> 3, g_rime_payloadlen, g_mytag);

          /* Add the next frame to the tail of the IOB queue */

          qtail->io_flink = iob;
          iob->io_flink   = NULL;

          /* Check tx result. */
#  warning Missing logic
        }

      /* Send the list of frames */

      return sixlowpan_send_frame(ieee, qhead);
#else
      nerr("ERROR: Packet too large: %d\n", len);
      nerr("       Cannot to be sent without fragmentation support\n");
      nerr("       dropping packet\n");

      return -E2BIG;
#endif
    }
  else
    {
      /* The packet does not need to be fragmented just copy the "payload"
       * and send in one frame.
       */

      /* Allocate an IOB to hold the frame, waiting if necessary. */

      iob = iob_alloc(false);
      DEBUGASSERT(iob != NULL);

      /* Format the single frame */
#  warning Missing logic

      /* Send the single frame */

      iob->io_flink = NULL;
      return sixlowpan_send_frame(ieee, iob);
    }
}

#endif /* CONFIG_NET_6LOWPAN */
