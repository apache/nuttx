/****************************************************************************
 * net/route/net_router.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <netinet/in.h>

#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "route/cacheroute.h"
#include "route/route.h"
#include "utils/utils.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
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
struct route_ipv4_match_s
{
  in_addr_t target;              /* Target IPv4 address on remote network */
#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  struct net_route_ipv4_s entry; /* Full entry from the IPv4 routing table */
#else
  in_addr_t router;              /* IPv4 address of router a local networks */
#endif
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  /* Only match prefix longer than prefixlen, equals to entry.netmask if we
   * have got a match (then we only find longer prefix later).
   * Range: -1 ~ 32
   */

  int8_t prefixlen;
#endif
};
#endif

#ifdef CONFIG_NET_IPv6
struct route_ipv6_match_s
{
  net_ipv6addr_t target;         /* Target IPv6 address on remote network */
#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  struct net_route_ipv6_s entry; /* Full entry from the IPv6 routing table */
#else
  net_ipv6addr_t router;         /* IPv6 address of router a local networks */
#endif
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  /* Only match prefix longer than prefixlen, equals to entry.netmask if we
   * have got a match (then we only find longer prefix later).
   * Range: -1 ~ 128
   */

  int16_t prefixlen;
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv4_match
 *
 * Description:
 *   Return 1 if the IPv4 route is available
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
static int net_ipv4_match(FAR struct net_route_ipv4_s *route, FAR void *arg)
{
  FAR struct route_ipv4_match_s *match =
                               (FAR struct route_ipv4_match_s *)arg;
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  int8_t prefixlen = (int8_t)net_ipv4_mask2pref(route->netmask);
#endif

  /* To match, the masked target addresses must be the same.  In the event
   * of multiple matches, only the first is returned.  There is not (yet) any
   * concept for the precedence of networks.
   */

  if (net_ipv4addr_maskcmp(route->target, match->target, route->netmask)
#ifdef CONFIG_ROUTE_LONGEST_MATCH
      && prefixlen > match->prefixlen
#endif
     )
    {
#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
      /* They match.. Copy the entire routing table entry */

      memcpy(&match->entry, route, sizeof(struct net_route_ipv4_s));
#else
      /* They match.. Copy the router address */

      net_ipv4addr_copy(match->router, route->router);
#endif
#ifdef CONFIG_ROUTE_LONGEST_MATCH
      /* Cache the prefix length */

      match->prefixlen = prefixlen;
#else
      return 1;
#endif
    }

  return 0;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: net_ipv6_match
 *
 * Description:
 *   Return 1 if the IPv6 route is available
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
static int net_ipv6_match(FAR struct net_route_ipv6_s *route, FAR void *arg)
{
  FAR struct route_ipv6_match_s *match =
                                (FAR struct route_ipv6_match_s *)arg;
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  int16_t prefixlen = (int16_t)net_ipv6_mask2pref(route->netmask);
#endif

  /* To match, the masked target addresses must be the same.  In the event
   * of multiple matches, only the first is returned.  There is not (yet) any
   * concept for the precedence of networks.
   */

  if (net_ipv6addr_maskcmp(route->target, match->target, route->netmask)
#ifdef CONFIG_ROUTE_LONGEST_MATCH
      && prefixlen > match->prefixlen
#endif
     )
    {
#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
      /* They match.. Copy the entire routing table entry */

      memcpy(&match->entry, route, sizeof(struct net_route_ipv6_s));
#else
      /* They match.. Copy the router address */

      net_ipv6addr_copy(match->router, route->router);
#endif
#ifdef CONFIG_ROUTE_LONGEST_MATCH
      /* Cache the prefix length */

      match->prefixlen = prefixlen;
#else
      return 1;
#endif
    }

  return 0;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Input Parameters:
 *   target    - An IPv4 address on a remote network to use in the lookup.
 *   router    - The address of router on a local network that can forward
 *               our packets to the target.
 *   prefixlen - The prefix length of previously matched routes (maybe on
 *               device), will only match prefix longer than prefixlen.
 *               Range: -1(match all) ~ 32(match none)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_ipv4_router(in_addr_t target, FAR in_addr_t *router,
                    int8_t prefixlen)
{
  struct route_ipv4_match_s match;
  int ret;

  /* Just early return for long prefix, maybe already got exact match. */

  if (prefixlen >= 32)
    {
      return -ENOENT;
    }

  /* Do not route the special broadcast IP address */

  if (net_ipv4addr_cmp(target, INADDR_BROADCAST))
    {
      return -ENOENT;
    }

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv4_match_s));
  net_ipv4addr_copy(match.target, target);
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  match.prefixlen = prefixlen;
#endif

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  /* First see if we can find a router entry in the cache */

  ret = net_foreachcache_ipv4(net_ipv4_match, &match);
  if (ret <= 0)
#endif
    {
      /* Not found in the cache.  Try to find a router entry with the
       * routing table that can forward to this address
       */

      ret = net_foreachroute_ipv4(net_ipv4_match, &match);
    }

  /* Did we find a route? */

#ifdef CONFIG_ROUTE_LONGEST_MATCH
  UNUSED(ret);
  if (match.prefixlen <= prefixlen)
#else
  if (ret <= 0)
#endif
    {
      /* No.. there is no route for this address */

      return -ENOENT;
    }

  /* We found a route. */

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  /* Add the route to the cache.  If the route is already in the cache, this
   * will update it to the most recently accessed.
   */

  net_addcache_ipv4(&match.entry);
#endif

  /* Return the router address. */

  net_ipv4addr_copy(*router, match.IPv4_ROUTER);
  return OK;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: net_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Input Parameters:
 *   target    - An IPv6 address on a remote network to use in the lookup.
 *   router    - The address of router on a local network that can forward
 *               our packets to the target.
 *   prefixlen - The prefix length of previously matched routes (maybe on
 *               device), will only match prefix longer than prefixlen.
 *               Range: -1(match all) ~ 128(match none)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int net_ipv6_router(const net_ipv6addr_t target, net_ipv6addr_t router,
                    int16_t prefixlen)
{
  struct route_ipv6_match_s match;
  int ret;

  /* Just early return for long prefix, maybe already got exact match. */

  if (prefixlen >= 128)
    {
      return -ENOENT;
    }

  /* Do not route to any the special IPv6 multicast addresses */

  if (target[0] == HTONS(0xff02))
    {
      return -ENOENT;
    }

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv6_match_s));
  net_ipv6addr_copy(match.target, target);
#ifdef CONFIG_ROUTE_LONGEST_MATCH
  match.prefixlen = prefixlen;
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  /* First see if we can find a router entry in the cache */

  ret = net_foreachcache_ipv6(net_ipv6_match, &match);
  if (ret <= 0)
#endif
    {
      /* Not found in the cache.  Try to find a router entry with the
       * routing table that can forward to this address
       */

      ret = net_foreachroute_ipv6(net_ipv6_match, &match);
    }

  /* Did we find a route? */

#ifdef CONFIG_ROUTE_LONGEST_MATCH
  UNUSED(ret);
  if (match.prefixlen <= prefixlen)
#else
  if (ret <= 0)
#endif
    {
      /* No.. there is no route for this address */

      return -ENOENT;
    }

  /* We found a route. */

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  /* Add the route to the cache.  If the route is already in the cache, this
   * will update it to the most recently accessed.
   */

  net_addcache_ipv6(&match.entry);
#endif

  /* Return the router address. */

  net_ipv6addr_copy(router, match.IPv6_ROUTER);
  return OK;
}
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
