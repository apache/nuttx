/****************************************************************************
 * net/ipforward/ipfwd_poll.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

  /* Make sure the there is something in buffer that is at least as large as
   * the IPv6_HDR.
   */

  if (dev->d_len > (IPv6_HDRLEN + NET_LL_HDRLEN(dev)))
    {
      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          /* There should be an IPv6 packet at the beginning of the buffer */

          ipv6 = IPv6BUF;
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
 *   devif_poll().
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

  flags = devif_conn_event(dev, IPFWD_POLL, dev->d_conncb);

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
