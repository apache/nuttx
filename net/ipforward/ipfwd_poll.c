/****************************************************************************
 * net/ipforward/ipfwd_poll.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>

#include "devif/devif.h"
#include "sixlowpan/sixlowpan.h"
#include "ipforward/ipforward.h"

#ifdef CONFIG_NET_IPFORWARD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfwd_packet_proto
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 (and
 *   other, non-standard packet radios) for now but this is a point where
 *   support for other conversions may be provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static int ipfwd_packet_proto(FAR struct net_driver_s *dev)
{
  FAR struct ipv6_hdr_s *ipv6;
  int llhdrlen = NET_LL_HDRLEN(dev);

  /* Make sure the there is something in buffer that is at least as large as
   * the IPv6_HDR.
   */

  if (dev->d_len > (IPv6_HDRLEN + llhdrlen))
    {
      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          /* There should be an IPv6 packet at the beginning of the buffer */

          ipv6 = (FAR struct ipv6_hdr_s *)&dev->d_buf[llhdrlen];
          if ((ipv6->vtc & IP_VERSION_MASK) == IPv6_VERSION)
            {
              /* Yes.. return the L2 protocol of the packet */

              return ipv6->proto;
            }
        }
    }

  return -EPROTO;
}
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Name: ipfwd_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 (and
 *   other, non-standard packet radios) for now but this is a point where
 *   support for other conversions may be provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static void ipfwd_packet_conversion(FAR struct net_driver_s *dev, int proto)
{
  if (dev->d_len > 0)
    {
      /* Check if this is a device served by 6LoWPAN */

      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_TCP
          if (proto == IP_PROTO_TCP)
            {
              /* Let 6LoWPAN convert IPv6 TCP output into radio frames. */

              sixlowpan_tcp_send(dev, dev, ipv6);
            }
          else
#endif
#ifdef CONFIG_NET_UDP
          if (proto == IP_PROTO_UDP)
            {
              /* Let 6LoWPAN convert IPv6 UDP output into radio frames. */

              sixlowpan_udp_send(dev, dev, ipv6);
            }
          else
#endif
#ifdef CONFIG_NET_ICMPv6
          if (proto == IP_PROTO_ICMP6)
            {
              /* Let 6LoWPAN convert IPv6 UDP output into radio frames. */

              sixlowpan_icmpv6_send(dev, dev, ipv6);
            }
          else
#endif
            {
              nwarn("WARNING: Unsupported protocol (%u). Packet dropped\n",
                    proto);
            }

          dev->d_len = 0;
        }
    }
}
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfwd_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll() and devif_timer().
 *
 ****************************************************************************/

void ipfwd_poll(FAR struct net_driver_s *dev)
{
  uint16_t flags;

  /* Setup for the callback (most of these do not apply) */

  dev->d_appdata = NULL;
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Perform the forwarding callbacks.  Returns the new set of flags.  If
   * the packet was forwarded, then the new set will be zero.
   */

  flags = devif_conn_event(dev, NULL, IPFWD_POLL, dev->d_conncb);

#ifdef CONFIG_NET_6LOWPAN
  if ((flags & DEVPOLL_MASK) == 0)
    {
      /* Get the L2 protocol of packet in the device's d_buf */

      int proto = ipfwd_packet_proto(dev);
      if (proto >= 0)
        {
          /* Perform any necessary conversions on the forwarded packet */

          ipfwd_packet_conversion(dev, proto);
        }
    }
#else
  UNUSED(flags);
#endif
}

#endif /* CONFIG_NET_ARP_SEND */
