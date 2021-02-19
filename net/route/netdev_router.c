/****************************************************************************
 * net/route/netdev_router.c
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
#include <string.h>
#include <errno.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "route/cacheroute.h"
#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
#  define IPv4_ROUTER entry.router
#else
#  define IPv4_ROUTER router
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
#  define IPv6_ROUTER entry.router
#else
#  define IPv6_ROUTER router
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
struct route_ipv4_devmatch_s
{
  FAR struct net_driver_s *dev;  /* The route must use this device */
  in_addr_t target;              /* Target IPv4 address on remote network */
#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  struct net_route_ipv4_s entry; /* Full entry from the IPv4 routing table */
#else
  in_addr_t router;              /* IPv4 address of router a local networks */
#endif
};
#endif

#ifdef CONFIG_NET_IPv6
struct route_ipv6_devmatch_s
{
  FAR struct net_driver_s *dev;  /* The route must use this device */
  net_ipv6addr_t target;         /* Target IPv4 address on remote network */
#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  struct net_route_ipv6_s entry; /* Full entry from the IPv6 routing table */
#else
  net_ipv6addr_t router;         /* IPv6 address of router a local networks */
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv4_devmatch
 *
 * Description:
 *   Return 1 if the IPv4 route is available on the device's network.
 *
 * Input Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int net_ipv4_devmatch(FAR struct net_route_ipv4_s *route,
                             FAR void *arg)
{
  FAR struct route_ipv4_devmatch_s *match =
    (FAR struct route_ipv4_devmatch_s *)arg;
  FAR struct net_driver_s *dev = match->dev;

  /* To match, (1) the masked target addresses must be the same, and (2) the
   * router address must like on the network provided by the device.
   *
   * In the event of multiple matches, only the first is returned.  There
   * not (yet) any concept for the precedence of networks.
   */

  if (net_ipv4addr_maskcmp(route->target, match->target, route->netmask) &&
      net_ipv4addr_maskcmp(route->router, dev->d_ipaddr, dev->d_netmask))
    {
#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
      /* They match.. Copy the entire routing table entry */

      memcpy(&match->entry, route, sizeof(struct net_route_ipv4_s));
#else
      /* They match.. Copy the router address */

      net_ipv4addr_copy(match->router, route->router);
#endif
      return 1;
    }

  return 0;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: net_ipv6_devmatch
 *
 * Description:
 *   Return 1 if the IPv6 route is available on the device's network.
 *
 * Input Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int net_ipv6_devmatch(FAR struct net_route_ipv6_s *route,
                             FAR void *arg)
{
  FAR struct route_ipv6_devmatch_s *match =
    (FAR struct route_ipv6_devmatch_s *)arg;
  FAR struct net_driver_s *dev = match->dev;

  /* To match, (1) the masked target addresses must be the same, and (2) the
   * router address must like on the network provided by the device.
   *
   * In the event of multiple matches, only the first is returned.  There
   * not (yet) any concept for the precedence of networks.
   */

  if (net_ipv6addr_maskcmp(route->target, match->target, route->netmask) &&
      net_ipv6addr_maskcmp(route->router, dev->d_ipv6addr,
                           dev->d_ipv6netmask))
    {
#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
      /* They match.. Copy the entire routing table entry */

      memcpy(&match->entry, route, sizeof(struct net_route_ipv6_s));
#else
      /* They match.. Copy the router address */

      net_ipv6addr_copy(match->router, route->router);
#endif
      return 1;
    }

  return 0;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv4_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Input Parameters:
 *   dev    - We are committed to using this device.
 *   target - An IPv4 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   A router address is always returned (which may just be, perhaps,
 *   device's default router address)
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void netdev_ipv4_router(FAR struct net_driver_s *dev, in_addr_t target,
                        FAR in_addr_t *router)
{
  struct route_ipv4_devmatch_s match;
  int ret;

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv4_devmatch_s));
  match.dev = dev;
  net_ipv4addr_copy(match.target, target);

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  /* First see if we can find a router entry in the cache */

  ret = net_foreachcache_ipv4(net_ipv4_devmatch, &match);
  if (ret <= 0)
#endif
    {
      /* Not found in the cache.  Try to find a router entry with the
       * routing table that can forward to this address
       */

      ret = net_foreachroute_ipv4(net_ipv4_devmatch, &match);
    }

  /* Did we find a route? */

  if (ret > 0)
    {
      /* We found a route. */

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
      /* Add the route to the cache.  If the route is already in the cache,
       * this will update it to the most recently accessed.
       */

      ret = net_addcache_ipv4(&match.entry);
#endif

      /* We Return the router address. */

      net_ipv4addr_copy(*router, match.IPv4_ROUTER);
    }
  else
    {
      /* There isn't a matching route.. fallback and use the default
       * router
       * of the device.
       */

      net_ipv4addr_copy(*router, dev->d_draddr);
    }
}
#endif

/****************************************************************************
 * Name: netdev_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv6_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Input Parameters:
 *   dev    - We are committed to using this device.
 *   target - An IPv6 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   A router address is always returned (which may just be, perhaps,
 *   device's default router address)
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void netdev_ipv6_router(FAR struct net_driver_s *dev,
                        FAR const net_ipv6addr_t target,
                        FAR net_ipv6addr_t router)
{
  struct route_ipv6_devmatch_s match;
  int ret;

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv6_devmatch_s));
  match.dev = dev;
  net_ipv6addr_copy(match.target, target);

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  /* First see if we can find a router entry in the cache */

  ret = net_foreachcache_ipv6(net_ipv6_devmatch, &match);
  if (ret <= 0)
#endif
    {
      /* Not found in the cache.  Try to find a router entry with the
       * routing table that can forward to this address
       */

      ret = net_foreachroute_ipv6(net_ipv6_devmatch, &match);
    }

  /* Did we find a route? */

  if (ret > 0)
    {
      /* We found a route. */

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
      /* Add the route to the cache.  If the route is already in the cache,
       * this will update it to the most recently accessed.
       */

      ret = net_addcache_ipv6(&match.entry);
#endif
      /* Return the router address. */

      net_ipv6addr_copy(router, match.IPv6_ROUTER);
    }
  else
    {
      /* There isn't a matching route.. fallback and use the default
       * router of the device.
       */

      net_ipv6addr_copy(router, dev->d_ipv6draddr);
    }
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
