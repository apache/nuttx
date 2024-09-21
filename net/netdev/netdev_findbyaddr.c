/****************************************************************************
 * net/netdev/netdev_findbyaddr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "utils/utils.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "route/route.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_prefixlen_findby_lipv4addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv4 address assigned to the network device.  Or any
 *             IPv4 address on the sub-net served by the network device.
 *   prefixlen - The length of matching prefix. Range: -1(no match) ~ 32
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static FAR struct net_driver_s *
netdev_prefixlen_findby_lipv4addr(in_addr_t lipaddr, FAR int8_t *prefixlen)
{
  FAR struct net_driver_s *dev;
  FAR struct net_driver_s *bestdev  = NULL;
  int8_t                   bestpref = -1;
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  int8_t len;
#endif

  /* Examine each registered network device */

  net_lock();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0 &&
          !net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY))
        {
#ifndef CONFIG_ROUTE_LONGEST_MATCH
          /* Yes.. check for an address match (under the netmask) */

          if (net_ipv4addr_maskcmp(dev->d_ipaddr, lipaddr,
                                   dev->d_netmask))
            {
              /* Its a match */

              bestdev  = dev;
              bestpref = 32; /* Regard as best (exact) match */
              break;
            }
#else
          /* Longest prefix flow: First, check for an exact address match */

          if (net_ipv4addr_cmp(dev->d_ipaddr, lipaddr))
            {
              /* It's an exact match */

              bestdev  = dev;
              bestpref = 32;
              break;
            }

          /* Then, check for an address match (under the netmask) */

          if (net_ipv4addr_maskcmp(dev->d_ipaddr, lipaddr,
                                   dev->d_netmask))
            {
              len = (int8_t)net_ipv4_mask2pref(dev->d_netmask);

              /* Regard current device as better if:
               * 1. It has longer prefix length
               * 2. It has the same prefix length but it has target address
               *    in the ARP cache (We don't have other information
               *    for the precedence of networks)
               */

              if (len > bestpref
#ifdef CONFIG_NET_ARP
                  || (len == bestpref && arp_find(lipaddr, NULL, dev) == OK)
#endif
                  )
                {
                  bestdev  = dev;
                  bestpref = len;
                }
            }
#endif /* CONFIG_ROUTE_LONGEST_MATCH */
        }
    }

  net_unlock();
  *prefixlen = bestpref;
  return bestdev;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: netdev_prefixlen_findby_lipv6addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv6 address assigned to the network device.  Or any
 *             IPv6 address on the sub-net served by the network device.
 *   prefixlen - The length of matching prefix. Range: -1(no match) ~ 128
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static FAR struct net_driver_s *
netdev_prefixlen_findby_lipv6addr(const net_ipv6addr_t lipaddr,
                                  FAR int16_t *prefixlen)
{
  FAR struct net_driver_s *dev;
  FAR struct net_driver_s *bestdev  = NULL;
  int16_t                  bestpref = -1;
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  FAR struct netdev_ifaddr6_s *ifaddr6;
  FAR struct neighbor_entry_s *ne;
  FAR struct net_driver_s     *hint;
  int16_t len;
#endif

  net_lock();

#ifdef CONFIG_ROUTE_LONGEST_MATCH
  /* Find a hint from neighbor table in case same prefix length exists on
   * multiple devices.
   */

  ne   = neighbor_findentry(lipaddr);
  hint = ne ? ne->ne_dev : NULL;
#endif

  /* Examine each registered network device */

  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0 && NETDEV_HAS_V6ADDR(dev))
        {
#ifndef CONFIG_ROUTE_LONGEST_MATCH
          /* Yes.. check for an address match (under the netmask) */

          if (NETDEV_V6ADDR_ONLINK(dev, lipaddr))
            {
              /* Its a match */

              bestdev  = dev;
              bestpref = 128; /* Regard as best (exact) match */
              break;
            }
#else
          /* Longest prefix flow: First, check for an exact address match */

          if (NETDEV_IS_MY_V6ADDR(dev, lipaddr))
            {
              /* It's an exact match */

              bestdev  = dev;
              bestpref = 128;
              break;
            }

          /* Then, check for an address match (under the netmask) */

          if ((ifaddr6 = netdev_ipv6_lookup(dev, lipaddr, true)) != NULL)
            {
              len = (int16_t)net_ipv6_mask2pref(ifaddr6->mask);

              /* Regard current device as better if:
               * 1. It has longer prefix length
               * 2. It has the same prefix length but it has target address
               *    in the neighbor cache (We don't have other information
               *    for the precedence of networks)
               */

              if (len > bestpref || (len == bestpref && hint == dev))
                {
                  bestdev  = dev;
                  bestpref = len;
                }
            }
#endif /* CONFIG_ROUTE_LONGEST_MATCH */
        }
    }

  net_unlock();
  *prefixlen = bestpref;
  return bestdev;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_findby_lipv4addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv4 address assigned to the network device.  Or any
 *             IPv4 address on the sub-net served by the network device.
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *netdev_findby_lipv4addr(in_addr_t lipaddr)
{
  int8_t prefixlen;
  return netdev_prefixlen_findby_lipv4addr(lipaddr, &prefixlen);
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: netdev_findby_lipv6addr
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Input Parameters:
 *   lipaddr - Local, IPv6 address assigned to the network device.  Or any
 *             IPv6 address on the sub-net served by the network device.
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *netdev_findby_lipv6addr(
                                      const net_ipv6addr_t lipaddr)
{
  int16_t prefixlen;
  return netdev_prefixlen_findby_lipv6addr(lipaddr, &prefixlen);
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: netdev_findby_ripv4addr
 *
 * Description:
 *   Find a previously registered network device by matching the remote
 *   IPv4 address that can be reached by the device.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection (used only if ripaddr is
 *             the broadcast address).
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *netdev_findby_ripv4addr(in_addr_t lipaddr,
                                                 in_addr_t ripaddr)
{
  FAR struct net_driver_s *dev;
  int8_t prefixlen;
#ifdef CONFIG_NET_ROUTE
  in_addr_t router;
  int ret;
#endif

  /* First, check if this is the broadcast IP address */

  if (net_ipv4addr_cmp(ripaddr, INADDR_BROADCAST))
    {
      /* Yes.. Check the local, bound address.  Is it INADDR_ANY? */

      if (net_ipv4addr_cmp(lipaddr, INADDR_ANY))
        {
          /* Yes.. In this case, I think we are supposed to send the
           * broadcast packet out ALL locally available networks.  I am not
           * sure of that and, in any event, there is nothing we can do
           * about that here.
           */

          return netdev_default();
        }
      else
        {
          /* Return the device associated with the local address */

          return netdev_findby_lipv4addr(lipaddr);
        }
    }

  /* Check if the address maps to a locally available network
   * Note: If longest prefix match is not enabled, prefixlen will be 32 if
   *       matched and it will disable further routing lookup.
   */

  dev = netdev_prefixlen_findby_lipv4addr(ripaddr, &prefixlen);

#ifdef CONFIG_NET_ROUTE
  /* If we have a routing table, then perhaps we can find the local
   * address of a router that can forward packets to the external network
   * with longer prefix.
   */

  ret = net_ipv4_router(ripaddr, &router, prefixlen);
  if (ret >= 0)
    {
      /* Success... try to find the network device associated with the local
       * router address
       */

      dev = netdev_findby_lipv4addr(router);
    }
#endif /* CONFIG_NET_ROUTE */

  /* Return the device we found. */

  if (dev)
    {
      return dev;
    }

  /* The above lookup will fail if the packet is being sent out of our
   * out subnet to a router and there is no routing information. Let's
   * try the default network device.
   */

  return netdev_default();
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: netdev_findby_ripv6addr
 *
 * Description:
 *   Find a previously registered network device by matching the remote
 *   IPv6 address that can be reached by the device.
 *
 * Input Parameters:
 *   lipaddr - Local, bound address of a connection (used only if ripaddr is
 *             a multicast address).
 *   ripaddr - Remote address of a connection to use in the lookup
 *
 * Returned Value:
 *   Pointer to driver on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *netdev_findby_ripv6addr(
                                 const net_ipv6addr_t lipaddr,
                                 const net_ipv6addr_t ripaddr)
{
  FAR struct net_driver_s *dev;
  int16_t prefixlen;
#ifdef CONFIG_NET_ROUTE
  net_ipv6addr_t router;
  int ret;
#endif

  /* First, check if this is the multicast IP address */

  if (net_is_addr_mcast(ripaddr))
    {
      /* Yes.. Check the local, bound address.  Is it the IPv6 unspecified
       * address?
       */

      if (net_ipv6addr_cmp(lipaddr, g_ipv6_unspecaddr))
        {
          /* Yes.. In this case, I think we are supposed to send the
           * broadcast packet out ALL locally available networks.  I am not
           * sure of that and, in any event, there is nothing we can do
           * about that here.
           */

          return netdev_default();
        }
      else
        {
          /* Return the device associated with the local address */

          return netdev_findby_lipv6addr(lipaddr);
        }
    }

  /* Check if the address maps to a locally available network
   * Note: If longest prefix match is not enabled, prefixlen will be 128 if
   *       matched and it will disable further routing lookup.
   */

  dev = netdev_prefixlen_findby_lipv6addr(ripaddr, &prefixlen);

#ifdef CONFIG_NET_ROUTE
  /* If we have a routing table, then perhaps we can find the local
   * address of a router that can forward packets to the external network
   * with longer prefix.
   */

  ret = net_ipv6_router(ripaddr, router, prefixlen);
  if (ret >= 0)
    {
      /* Success... try to find the network device associated with the local
       * router address
       */

      dev = netdev_findby_lipv6addr(router);
    }
#endif /* CONFIG_NET_ROUTE */

  /* Return the device we found. */

  if (dev)
    {
      return dev;
    }

  /* The above lookup will fail if the packet is being sent out of our
   * out subnet to a router and there is no routing information. Let's
   * try the default network device.
   */

  return netdev_default();
}
#endif /* CONFIG_NET_IPv6 */
