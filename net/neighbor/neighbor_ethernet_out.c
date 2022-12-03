/****************************************************************************
 * net/neighbor/neighbor_ethernet_out.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/ethernet.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/neighbor.h>

#include "devif/devif.h"
#include "route/route.h"
#include "icmpv6/icmpv6.h"
#include "neighbor/neighbor.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Support for MLD multicast addresses.
 *
 * Well-known ethernet multicast address:
 *
 * ADDRESS           TYPE   USAGE
 * 01-00-0c-cc-cc-cc 0x0802 CDP (Cisco Discovery Protocol), VTP (Virtual
 *                          Trunking Protocol)
 * 01-00-0c-cc-cc-cd 0x0802 Cisco Shared Spanning Tree Protocol Address
 * 01-80-c2-00-00-00 0x0802 Spanning Tree Protocol (for bridges) IEEE 802.1D
 * 01-80-c2-00-00-02 0x0809 Ethernet OAM Protocol IEEE 802.3ah
 * 01-00-5e-xx-xx-xx 0x0800 IPv4 IGMP Multicast Address
 * 33-33-00-00-00-00 0x86DD IPv6 Neighbor Discovery
 * 33-33-xx-xx-xx-xx 0x86DD IPv6 Multicast Address (RFC3307)
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: neighbor_ethernet_out
 *
 * Description:
 *   This function should be called before sending out an IPv6 packet. The
 *   function checks the destination IPv6 address of the IPv6 packet to see
 *   what Ethernet MAC address that should be used as a destination MAC
 *   address on the Ethernet.
 *
 *   If the destination IPv6 address is in the local network (determined
 *   by logical ANDing of netmask and our IPv6 address), the function
 *   checks the Neighbor Table to see if an entry for the destination IPv6
 *   address is found.  If so, an Ethernet header is pre-pended at the
 *   beginning of the packet and the function returns.
 *
 *   If no Neighbor Table entry is found for the destination IPv6 address,
 *   the packet in the d_buf is replaced by an ICMPv6 Neighbor Solicit
 *   request packet for the IPv6 address. The IPv6 packet is dropped and
 *   it is assumed that the higher level protocols (e.g., TCP) eventually
 *   will retransmit the dropped packet.
 *
 *   Upon return in either the case, a packet to be sent is present in the
 *   d_buf buffer and the d_len field holds the length of the Ethernet
 *   frame that should be transmitted.
 *
 ****************************************************************************/

void neighbor_ethernet_out(FAR struct net_driver_s *dev)
{
  FAR struct eth_hdr_s *eth = ETHBUF;
  FAR struct ipv6_hdr_s *ip = IPv6BUF;
  struct neighbor_addr_s laddr;

  /* Skip sending Neighbor Solicitations when the frame to be transmitted was
   * written into a packet socket.
   */

  if (IFF_IS_NOARP(dev->d_flags))
    {
      /* Clear the indication and let the packet continue on its way. */

      IFF_CLR_NOARP(dev->d_flags);
      return;
    }

  /* Find the destination IPv6 address in the Neighbor Table and construct
   * the Ethernet header. If the destination IPv6 address isn't on the local
   * network, we use the default router's IPv6 address instead.
   *
   * If no Neighbor Table entry is found, we overwrite the original IPv6
   * packet with an Neighbor Solicitation Request for the IPv6 address.
   */

  /* First check if destination isn't IPv6 multicast address. */

  if (!net_is_addr_mcast(ip->destipaddr))
    {
      net_ipv6addr_t ipaddr;

      /* Check if the destination address is on the local network. */

      if (!net_ipv6addr_maskcmp(ip->destipaddr, dev->d_ipv6addr,
                                dev->d_ipv6netmask))
        {
          /* Destination address is not on the local network */

#ifdef CONFIG_NET_ROUTE
          /* We have a routing table.. find the correct router to use in
           * this case (or, as a fall-back, use the device's default router
           * address).  We will use the router IPv6 address instead of the
           * destination address when determining the MAC address.
           */

          netdev_ipv6_router(dev, ip->destipaddr, ipaddr);
#else
          /* Use the device's default router IPv6 address instead of the
           * destination address when determining the MAC address.
           */

          net_ipv6addr_copy(ipaddr, dev->d_ipv6draddr);
#endif
        }
      else
        {
          /* Else, we use the destination IPv6 address. */

          net_ipv6addr_copy(ipaddr, ip->destipaddr);
        }

      /* Check if we already have this destination address in the
       * Neighbor Table.
       */

      if (neighbor_lookup(ipaddr, &laddr) < 0)
        {
#ifdef CONFIG_NET_ICMPv6
           ninfo("IPv6 Neighbor solicitation for IPv6\n");

          /* The destination address was not in our Neighbor Table, so we
           * overwrite the IPv6 packet with an ICMPv6 Neighbor Solicitation
           * message.
           */

          icmpv6_solicit(dev, ipaddr);
#else
          /* What to do here? We need the laddr, but no way to get it. */

          nerr("ERROR: IPv6 needs link layer address for ethernet.\n");
          DEBUGPANIC();
          return;
#endif
        }
    }

  /* Build an Ethernet header. */

  if (net_is_addr_mcast(ip->destipaddr))
    {
      eth->dest[0] = eth->dest[1] = 0x33;
      memcpy(&eth->dest[2], &ip->destipaddr[6], 4);
    }
  else
    {
      memcpy(eth->dest, laddr.u.na_ethernet.ether_addr_octet,
             ETHER_ADDR_LEN);
    }

  /* Finish populating the Ethernet header */

  memcpy(eth->src, dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);
  eth->type  = HTONS(ETHTYPE_IP6);

  /* Add the size of the layer layer header to the total size of the
   * outgoing packet.
   */

  dev->d_len += netdev_ipv6_hdrlen(dev);
  ninfo("Outgoing IPv6 Packet length: %d (%d)\n",
          dev->d_len, (ip->len[0] << 8) | ip->len[1]);
}
