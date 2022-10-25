/****************************************************************************
 * net/route/net_del_ramroute.c
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
#include <debug.h>

#include <arpa/inet.h>
#include <nuttx/net/ip.h>

#include "route/ramroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
struct route_match_ipv4_s
{
  FAR struct net_route_ipv4_s *prev;     /* Predecessor in the list */
  in_addr_t                    target;   /* The target IP address to match */
  in_addr_t                    netmask;  /* The network mask to match */
};
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
struct route_match_ipv6_s
{
  FAR struct net_route_ipv6_s *prev;     /* Predecessor in the list */
  net_ipv6addr_t               target;   /* The target IP address to match */
  net_ipv6addr_t               netmask;  /* The network mask to match */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_match_ipv4
 *
 * Description:
 *   Return 1 if the route is available
 *
 * Input Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
static int net_match_ipv4(FAR struct net_route_ipv4_s *route, FAR void *arg)
{
  FAR struct route_match_ipv4_s *match =
                    (FAR struct route_match_ipv4_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  net_ipv4_dumproute("Comparing", route);
  ninfo("With:\n");
  ninfo("  target=%08" PRIx32 " netmask=%08" PRIx32 "\n",
        HTONL(match->target), HTONL(match->netmask));

  if (net_ipv4addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv4addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. Remove the entry from the routing table */

      if (match->prev)
        {
          ramroute_ipv4_remafter(
                         (FAR struct net_route_ipv4_entry_s *)match->prev,
                         &g_ipv4_routes);
        }
      else
        {
          ramroute_ipv4_remfirst(&g_ipv4_routes);
        }

      /* And free the routing table entry by adding it to the free list */

      net_freeroute_ipv4(route);

      /* Return a non-zero value to terminate the traversal */

      return 1;
    }

  /* Next time we are here, this will be the previous entry */

  match->prev = route;
  return 0;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
static int net_match_ipv6(
               FAR struct net_route_ipv6_s *route, FAR void *arg)
{
  FAR struct route_match_ipv6_s *match =
                     (FAR struct route_match_ipv6_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  net_ipv6_dumproute("Comparing", route);
  ninfo("With:\n");
  ninfo("  target:  %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        HTONS(match->target[0]),  HTONS(match->target[1]),
        HTONS(match->target[2]),  HTONS(match->target[3]),
        HTONS(match->target[4]),  HTONS(match->target[5]),
        HTONS(match->target[6]),  HTONS(match->target[7]));
  ninfo("  netmask: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        HTONS(match->netmask[0]), HTONS(match->netmask[1]),
        HTONS(match->netmask[2]), HTONS(match->netmask[3]),
        HTONS(match->netmask[4]), HTONS(match->netmask[5]),
        HTONS(match->netmask[6]), HTONS(match->netmask[7]));

  if (net_ipv6addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv6addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. Remove the entry from the routing table */

      if (match->prev)
        {
          ramroute_ipv6_remafter(
                     (FAR struct net_route_ipv6_entry_s *)match->prev,
                     &g_ipv6_routes);
        }
      else
        {
          ramroute_ipv6_remfirst(&g_ipv6_routes);
        }

      /* And free the routing table entry by adding it to the free list */

      net_freeroute_ipv6(route);

      /* Return a non-zero value to terminate the traversal */

      return 1;
    }

  /* Next time we are here, this will be the previous entry */

  match->prev = route;
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_delroute_ipv4 and net_delroute_ipv6
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
int net_delroute_ipv4(in_addr_t target, in_addr_t netmask)
{
  struct route_match_ipv4_s match;

  /* Set up the comparison structure */

  match.prev = NULL;
  net_ipv4addr_copy(match.target, target);
  net_ipv4addr_copy(match.netmask, netmask);

  /* Then remove the entry from the routing table */

  return net_foreachroute_ipv4(net_match_ipv4, &match) ? OK : -ENOENT;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
int net_delroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask)
{
  struct route_match_ipv6_s match;

  /* Set up the comparison structure */

  match.prev = NULL;
  net_ipv6addr_copy(match.target, target);
  net_ipv6addr_copy(match.netmask, netmask);

  /* Then remove the entry from the routing table */

  return net_foreachroute_ipv6(net_match_ipv6, &match) ? OK : -ENOENT;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE  */
