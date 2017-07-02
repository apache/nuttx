/****************************************************************************
 * net/devif/ipv6_forward.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>

#include "netdev/netdev.h"
#include "sixlowpan/sixlowpan.h"
#include "devif/devif.h"

#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_IPv6)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 for now
 *   but this is a point where support for other conversions may be
 *   provided.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static int ipv6_packet_conversion(FAR struct net_driver_s *dev,
                                  FAR struct net_driver_s *fwddev,
                                  FAR struct ipv6_hdr_s *ipv6)
{
#ifdef CONFIG_NET_MULTILINK
  /* Handle the case where multiple link layer protocols are supported */

  if (dev->d_len > 0 && fwddev->d_lltype == NET_LL_IEEE802154)
#else
  if (dev->d_len > 0)
#endif
    {
#ifdef CONFIG_NET_TCP
      if (ipv6->proto == IP_PROTO_TCP)
        {
          /* Let 6LoWPAN convert IPv6 TCP output into IEEE802.15.4 frames. */

          sixlowpan_tcp_send(dev, fwddev, ipv6);
        }
      else
#endif
#ifdef CONFIG_NET_UDP
      if (ipv6->proto == IP_PROTO_UDP)
        {
          /* Let 6LoWPAN convert IPv6 UDP output into IEEE802.15.4 frames. */

          sixlowpan_udp_send(dev, fwddev, ipv6);
        }
      else
#endif
        {
          /* Otherwise, we will have to drop the packet */

          nwarn("WARNING: Dropping.  Unsupported 6LoWPAN protocol: %d\n",
                ipv6->proto);

#ifdef CONFIG_NET_STATISTICS
          g_netstats.ipv6.drop++;
#endif
        }

      dev->d_len = 0;
      return OK;
    }

  return -EPFNOSUPPORT;
}
#else
# define ipv6_packet_conversion(dev, ipv6)
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_forward
 *
 * Description:
 *   This function is called from ipv6_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device (or sent back out the same device)
 *   depending configuration, routing table information, and the IPv6
 *   networks served by various network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv6_input.
 *   - ipv6 points to the IPv6 header with dev->d_buf.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv6_input()) should drop the packet.
 *
 ****************************************************************************/

int ipv6_forward(FAR struct net_driver_s *dev, FAR struct ipv6_hdr_s *ipv6)
{
  /* Multiple network devices */

  FAR struct net_driver_s *fwddev;
  int ret;

  /* Search for a device that can forward this packet.  This is a trivial
   * serch if there is only a single network device (CONFIG_NETDEV_MULTINIC
   * not defined).  But netdev_findby_ipv6addr() will still assure
   * routability in that case.
   */

#ifdef CONFIG_NETDEV_MULTINIC
  fwddev = netdev_findby_ipv6addr(ipv6->srcipaddr, ipv6->destipaddr);
#else
  fwddev = netdev_findby_ipv6addr(ipv6->destipaddr);
#endif
  if (fwddev == NULL)
    {
      nwarn("WARNING: Not routable\n");
      return (ssize_t)-ENETUNREACH;
    }

#if defined(CONFIG_NETDEV_MULTINIC)
  /* Check if we are forwarding on the same device that we received the
   * packet from.
   */

  if (fwddev != dev)
    {
      /* Perform any necessary packet conversions. */

      ret = ipv6_packet_conversion(dev, fwddev, ipv6);
      if (ret < 0)
        {
          /* Extract the IPv6 + L3 header; Move the data payload to an IOB
           * chain.
           */

          /* Notify the forwarding device that TX data is available */

          /* Set up to send the packet when the selected device polls for TX
           * data.
           */

          /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
           * - source MAC, the MAC of the current device.
           * - dest MAC, the MAC associated with the destination IPv6 adress.
           *   This will involve ICMPv6 and Neighbor Discovery.
           */

          /* Return success with dev->d_len = 0 */

#  warning Missing logic
          nwarn("WARNING: Packet forwarding not yet supported "
                          "across different devices\n");
          return -ENOSYS;
        }
    }
  else
#endif /* CONFIG_NETDEV_MULTINIC */

#if defined(CONFIG_NET_6LOWPAN) /* REVISIT:  Currently only suport for
                                   * 6LoWPAN */
    {
      /* Single network device */

      /* Perform any necessary packet conversions.  If the packet was handled
       * via a backdoor path (or dropped), then dev->d_len will be zero.  If
       * the packet needs to be forwarded in the normal manner then
       * dev->d_len will be unchanged.
       */

      ret = ipv6_packet_conversion(dev, dev, ipv6);
      if (ret < 0)
        {
#ifdef CONFIG_NET_ETHERNET
          /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
           * - source MAC, the MAC of the current device.
           * - dest MAC, the MAC associated with the destination IPv6 adress.
           *   This  will involve ICMPv6 and Neighbor Discovery.
           */

          /* Correct dev->d_buf by adding back the L1 header length */
#endif

          /* Nothing other 6LoWPAN forwarding is currently handled and that
           * case was dealt with in ipv6_packet_conversion().
           */

#  warning Missing logic
          nwarn("WARNING: Packet forwarding supported only for 6LoWPAN\n");
          return -ENOSYS;
        }
    }

#else /* CONFIG_NET_6LOWPAN */
    {
      nwarn("WARNING: Packet forwarding not supported in this configuration\n");
      return -ENOSYS;
    }
#endif /* CONFIG_NET_6LOWPAN */

  /* Return success.  ipv6_input will return to the network driver with
   * dev->d_len set to the packet size and the network driver will perform
   * the transfer.
   */

  return OK;
}

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NET_IPv6 */
