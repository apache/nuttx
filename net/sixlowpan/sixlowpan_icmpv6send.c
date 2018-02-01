/****************************************************************************
 * net/sixlowpan/sixlowpan_icmpv6send.c
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
 *   dev    - An instance of nework device state structure
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
          nwarn("WARNING: Expected ICMPv6 protoype: %u vs %s\n",
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

          /* Get the IPv6 + ICMPv6 combined header length.  NOTE:  This header
           * size includes only the common 32-bit header at the beginning of
           * each ICMPv6 message.
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

              (void)sixlowpan_queue_frames(
                      (FAR struct radio_driver_s *)fwddev,
                      &ipv6icmpv6->ipv6, buf, buflen, &destmac);
            }
        }
    }

drop:
  dev->d_len = 0;
}

#endif /* CONFIG_NET_6LOWPAN && CONFIG_NET_ICMPv6 */
