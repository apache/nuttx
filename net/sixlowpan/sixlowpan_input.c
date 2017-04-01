/****************************************************************************
 * net/sixlowpan/sixlowpan_input.c
 * 6lowpan implementation (RFC4944 and draft-ietf-6lowpan-hc-06)
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives in large part from Contiki:
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "nuttx/net/netdev.h"
#include "nuttx/net/ip.h"
#include "nuttx/net/sixlowpan.h"

#ifdef CONFIG_NET_PKT
#  include "pkt/pkt.h"
#endif

#include "sixlowpan/sixlowpan_internal.h"

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Success return values from sixlowpan_frame_process */

#define INPUT_PARTIAL  0 /* Frame processed successful, packet incomplete */
#define INPUT_COMPLETE 1 /* Frame processed successful, packet complete */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sixlowpan_isbroadcast
 *
 * Description:
 *   Return the address length associated with a 2-bit address mode
 *
 * Input parameters:
 *   addrmode - The address mode
 *
 * Returned Value:
 *   The address length associated with the address mode.
 *
 ****************************************************************************/

static bool sixlowpan_isbroadcast(uint8_t mode, FAR uint8_t *addr)
{
  int i = ((mode == FRAME802154_SHORTADDRMODE) ? 2 : 8);

  while (i-- > 0)
    {
      if (addr[i] != 0xff)
        {
          return false;
        }
    }

  return true;
}

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

  g_pktattrs[PACKETBUF_ATTR_NETWORK_ID] = ipv6->proto;

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

  g_pktattrs[PACKETBUF_ATTR_CHANNEL] = attr;
}

/****************************************************************************
 * Name: sixlowpan_frame_process
 *
 * Description:
 *   Process an incoming 6loWPAN frame in 'iob'.
 *
 *   If its a FRAG1 or a non-fragmented frame we first uncompress the IP
 *   header. The 6loWPAN payload and possibly the uncompressed IP header
 *   are then copied into d_buf.  An indication is returned if the packet
 *   in d_buf is complete (i.e., non-fragmented frame or and the last
 *   FRAGN frame).
 *
 *   NOTE: We do not check for overlapping sixlowpan fragments (that is a
 *   SHALL in the RFC 4944 and should never happen)
 *
 * Input Parameters:
 *   ieee - The IEEE802.15.4 MAC network driver interface.
 *   iob  - The IOB containing the frame.
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 ****************************************************************************/

static int sixlowpan_frame_process(FAR struct ieee802154_driver_s *ieee,
                                   FAR struct iob_s *iob)
{
  /* REVISIT: To be provided */
  return -ENOSYS;
}

/****************************************************************************
 * Function: sixlowpan_dispatch
 *
 * Description:
 *   Inject the packet in d_buf into the network for normal packet processing.
 *
 * Parameters:
 *   ieee - The IEEE802.15.4 MAC network driver interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sixlowpan_dispatch(FAR struct ieee802154_driver_s *ieee)
{
#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the packet tap */

  ninfo("Packet tap\n");
  pkt_input(&ieee->i_dev);
#endif

  /* We only accept IPv6 packets. */

  ninfo("Iv6 packet dispatch\n");
  NETDEV_RXIPV6(&ieee->i_dev);

  /* Give the IPv6 packet to the network layer.  NOTE:  If there is a
   * problem with IPv6 header, it will be silently dropped and d_len will
   * be set to zero.  Oddly, ipv6_input() will return OK in this case.
   */

  return ipv6_input(&ieee->i_dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_input
 *
 * Description:
 *   Process an incoming 6loWPAN frame.
 *
 *   This function is called when the device driver has received a 6loWPAN
 *   frame from the network. The frame from the device driver must be
 *   provided in a IOB present in the i_framelist:  The frame data is in the
 *   IOB io_data[] buffer and the length of the frame is in the IOB io_len
 *   field.  Only a single IOB is expected in the i_framelist.  This incoming
 *   data will be processed one frame at a time.
 *
 *   An non-NULL d_buf of size CONFIG_NET_6LOWPAN_MTU must also be provided.
 *   The frame will be decompressed and placed in the d_buf. Fragmented
 *   packets will also be reassembled in the d_buf as they are received
 *   (meaning for the driver, that two packet buffers are required:  One for
 *   reassembly of RX packets and one used for TX polling).
 *
 *   After each frame is processed into d_buf, the IOB is removed and
 *   deallocated.  i_framelist will be nullified.  If reassembly is
 *   incomplete, this function will return to called with i_framelist
 *   equal to NULL.  The partially reassembled packet must be preserved by
 *   the IEEE802.15.4 MAC and provided again when the next frame is
 *   received.
 *
 *   When the packet in the d_buf is fully reassembled, it will be provided
 *   to the network as with any other received packet.  d_len will be set
 *   the the length of the uncompressed, reassembled packet.
 *
 *   After the network processes the packet, d_len will be set to zero.
 *   Network logic may also decide to send a response to the packet.  In
 *   that case, the outgoing network packet will be placed in d_buf the
 *   d_buf and d_len will be set to a non-zero value.  That case is handled
 *   by this function.
 *
 *   If that case occurs, the packet will be converted to a list of
 *   compressed and possibly fragmented frames in i_framelist as with other
 *   TX operations.
 *
 *   So from the standpoint of the IEEE802.15.4 MAC driver, there are two
 *   possible results:  (1) i_framelist is NULL meaning that the frame
 *   was fully processed and freed, or (2) i_framelist is non-NULL meaning
 *   that there are outgoing frame(s) to be sent.
 *
 * Input Parameters:
 *   ieee - The IEEE802.15.4 MAC network driver interface.
 *
 * Returned Value:
 *   Ok is returned on success; Othewise a negated errno value is returned.
 *
 ****************************************************************************/

int sixlowpan_input(FAR struct ieee802154_driver_s *ieee)
{
  int ret = -EINVAL;

  DEBUGASSERT(ieee != NULL && !FRAME_IOB_EMPTY(ieee));

  /* Verify that an IOB is provided in the device structure */

  while (!FRAME_IOB_EMPTY(ieee))
    {
      FAR struct iob_s *iob;

      /* Remove the IOB containing the frame from the device structure */

      FRAME_IOB_REMOVE(ieee, iob);
      DEBUGASSERT(iob != NULL);

      /* Process the frame, decompressing it into the packet buffer */

      ret = sixlowpan_frame_process(ieee, iob);

      /* Was the frame successfully processed? Is the packet in d_buf fully
       * reassembled?
       */

      if (ret == INPUT_COMPLETE)
        {
          /* Inject the uncompressed, reassembled packet into the network */

          ret = sixlowpan_dispatch(ieee);
          if (ret >= 0)
            {
              /* Check if this resulted in a request to send an outgoing
               * packet.
               */

              if (ieee->i_dev.d_len > 0)
                {
                  FAR struct ipv6_hdr_s *ipv6hdr;
                  struct rimeaddr_s destmac;

                  /* The IPv6 header followed by TCP or UDP headers should
                   * lie at the beginning of d_buf since there is no link
                   * layer protocol header.
                   */

                  ipv6hdr = (FAR struct ipv6_hdr_s *)(ieee->i_dev.d_buf);

                  /* Get the Rime MAC address of the destination */
#warning Missing logic

                  /* Convert the outgoing packet into a frame list. */

                  ret = sixlowpan_queue_frames(ieee, ipv6hdr, ieee->i_dev.d_buf,
                                               ieee->i_dev.d_len, &destmac);
                  ieee->i_dev.d_len = 0;
                }
            }
        }
    }

  return ret;
}

#endif /* CONFIG_NET_6LOWPAN */
