/****************************************************************************
 * net/sixlowpan/sixlowpan_icmpv6send.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/ip.h>

#include "icmpv6/icmpv6.h"
#include "sixlowpan/sixlowpan_internal.h"
#include "sixlowpan/sixlowpan.h"

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_NET_ICMPv6)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sixlowpan_icmpv6_send
 *
 * Description:
 *   Handles forwarding a ICMPv6 packet via 6LoWPAN.  This is currently only
 *   used by the IPv6 forwarding logic.
 *
 * Input Parameters:
 *   dev    - An instance of network device state structure
 *   fwddev - The network device used to send the data.  This will be the
 *            same device except for the IP forwarding case where packets
 *            are sent across devices.
 *   ipv6   - A pointer to the IPv6 header in dev->d_buf which lies AFTER
 *            the L1 header.  NOTE: dev->d_len must have been decremented
 *            by the size of any preceding MAC header.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void sixlowpan_icmpv6_send(FAR struct net_driver_s *dev,
                           FAR struct net_driver_s *fwddev,
                           FAR struct ipv6_hdr_s *ipv6)
{
  FAR struct ipv6icmp_hdr_s *ipv6icmpv6 = (FAR struct ipv6icmp_hdr_s *)ipv6;

  /* Double check */

  DEBUGASSERT(dev != NULL && dev->d_len > 0 && fwddev != NULL);

  ninfo("d_len %u\n", dev->d_len);

  if (dev != NULL && dev->d_len > 0)
    {
      sixlowpan_dumpbuffer("Outgoing ICMPv6 packet",
                           (FAR const uint8_t *)ipv6icmpv6, dev->d_len);

      /* The ICMPv6 data payload should follow the IPv6 header plus the
       * protocol header.
       */

      if (ipv6icmpv6->ipv6.proto != IP_PROTO_ICMP6)
        {
          nwarn("WARNING: Expected ICMPv6 prototype: %u vs %u\n",
                ipv6icmpv6->ipv6.proto, IP_PROTO_ICMP6);
        }
      else
        {
          struct netdev_varaddr_s destmac;
          FAR uint8_t *buf;
          uint16_t hdrlen;
          uint16_t buflen;
          int ret;

          /* Get the IEEE 802.15.4 MAC address of the next hop. */

          ret = sixlowpan_nexthopaddr((FAR struct radio_driver_s *)fwddev,
                                      ipv6icmpv6->ipv6.destipaddr, &destmac);
          if (ret < 0)
            {
              nerr("ERROR: Failed to get dest MAC address: %d\n", ret);
              goto drop;
            }

          /* Get the IPv6 + ICMPv6 combined header length.  NOTE:  This
           * header size includes only the common 32-bit header at the
           * beginning of each ICMPv6 message.
           */

          hdrlen = IPv6_HDRLEN + ICMPv6_HDRLEN;

          /* Drop the packet if the buffer length is less than this. */

          if (hdrlen > dev->d_len)
            {
              nwarn("WARNING:  Dropping small ICMPv6 packet: %u < %u\n",
                    buflen, hdrlen);
            }
          else
            {
              /* Convert the outgoing packet into a frame list. */

              buf    = (FAR uint8_t *)ipv6 + hdrlen;
              buflen = dev->d_len - hdrlen;

              sixlowpan_queue_frames(
                      (FAR struct radio_driver_s *)fwddev,
                      &ipv6icmpv6->ipv6, buf, buflen, &destmac);
            }
        }
    }

drop:
  dev->d_len = 0;
}

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_ICMPv6 */
