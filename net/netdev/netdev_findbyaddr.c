/****************************************************************************
 * net/netdev/netdev_findbyaddr.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "utils/utils.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "route/route.h"
#include "netdev/netdev.h"

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
  FAR struct net_driver_s *dev;

  /* Examine each registered network device */

  net_lock();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0 &&
          !net_ipv4addr_cmp(dev->d_ipaddr, INADDR_ANY))
        {
          /* Yes.. check for an address match (under the netmask) */

          if (net_ipv4addr_maskcmp(dev->d_ipaddr, lipaddr,
                                   dev->d_netmask))
            {
              /* Its a match */

              net_unlock();
              return dev;
            }
        }
    }

  /* No device with the matching address found */

  net_unlock();
  return NULL;
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
  FAR struct net_driver_s *dev;

  /* Examine each registered network device */

  net_lock();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0 &&
          !net_ipv6addr_cmp(dev->d_ipv6addr, g_ipv6_unspecaddr))
        {
          /* Yes.. check for an address match (under the netmask) */

          if (net_ipv6addr_maskcmp(dev->d_ipv6addr, lipaddr,
                                   dev->d_ipv6netmask))
            {
              /* Its a match */

              net_unlock();
              return dev;
            }
        }
    }

  /* No device with the matching address found */

  net_unlock();
  return NULL;
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
  struct net_driver_s *dev;
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

  /* Check if the address maps to a locally available network */

  dev = netdev_findby_lipv4addr(ripaddr);
  if (dev)
    {
      return dev;
    }

  /* No.. The address lies on an external network */

#ifdef CONFIG_NET_ROUTE
  /* If we have a routing table, then perhaps we can find the local
   * address of a router that can forward packets to the external network.
   */

  ret = net_ipv4_router(ripaddr, &router);
  if (ret >= 0)
    {
      /* Success... try to find the network device associated with the local
       * router address
       */

      dev = netdev_findby_lipv4addr(router);
      if (dev)
        {
          return dev;
        }
    }
#endif /* CONFIG_NET_ROUTE */

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
  struct net_driver_s *dev;
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

  /* Check if the address maps to a locally available network */

  dev = netdev_findby_lipv6addr(ripaddr);
  if (dev)
    {
      return dev;
    }

  /* No.. The address lies on an external network */

#ifdef CONFIG_NET_ROUTE
  /* If we have a routing table, then perhaps we can find the local
   * address of a router that can forward packets to the external network.
   */

  ret = net_ipv6_router(ripaddr, router);
  if (ret >= 0)
    {
      /* Success... try to find the network device associated with the local
       * router address
       */

      dev = netdev_findby_lipv6addr(router);
      if (dev)
        {
          return dev;
        }
    }
#endif /* CONFIG_NET_ROUTE */

  /* The above lookup will fail if the packet is being sent out of our
   * out subnet to a router and there is no routing information. Let's
   * try the default network device.
   */

  return netdev_default();
}
#endif /* CONFIG_NET_IPv6 */
