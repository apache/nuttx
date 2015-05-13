/****************************************************************************
 * net/route/net_delroute.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/net/ip.h>

#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
struct route_match_s
{
  FAR struct net_route_s *prev;     /* Predecessor in the list */
  in_addr_t               target;   /* The target IP address to match */
  in_addr_t               netmask;  /* The network mask to match */
};
#endif

#ifdef CONFIG_NET_IPv6
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
 * Function: net_match
 *
 * Description:
 *   Return 1 if the route is available
 *
 * Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int net_match(FAR struct net_route_s *route, FAR void *arg)
{
  FAR struct route_match_s *match = ( FAR struct route_match_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  if (net_ipv4addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv4addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. Remove the entry from the routing table */

      if (match->prev)
        {
          (void)sq_remafter((FAR sq_entry_t *)match->prev,
                            (FAR sq_queue_t *)&g_routes);
        }
      else
        {
          (void)sq_remfirst((FAR sq_queue_t *)&g_routes);
        }

      /* And free the routing table entry by adding it to the free list */

      net_freeroute(route);

      /* Return a non-zero value to terminate the traversal */

      return 1;
    }

  /* Next time we are here, this will be the previous entry */

  match->prev = route;
  return 0;
}
#endif

#ifdef CONFIG_NET_IPv6
static int net_match_ipv6(FAR struct net_route_ipv6_s *route, FAR void *arg)
{
  FAR struct route_match_ipv6_s *match = ( FAR struct route_match_ipv6_s *)arg;

  /* To match, the masked target address must be the same, and the masks
   * must be the same.
   */

  if (net_ipv6addr_maskcmp(route->target, match->target, match->netmask) &&
      net_ipv6addr_cmp(route->netmask, match->netmask))
    {
      /* They match.. Remove the entry from the routing table */

      if (match->prev)
        {
          (void)sq_remafter((FAR sq_entry_t *)match->prev,
                            (FAR sq_queue_t *)&g_routes_ipv6);
        }
      else
        {
          (void)sq_remfirst((FAR sq_queue_t *)&g_routes_ipv6);
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
 * Function: net_delroute
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_delroute(in_addr_t target, in_addr_t netmask)
{
  struct route_match_s match;

  /* Set up the comparison structure */

  match.prev = NULL;
  net_ipv4addr_copy(match.target, target);
  net_ipv4addr_copy(match.netmask, netmask);

  /* Then remove the entry from the routing table */

  return net_foreachroute(net_match, &match) ? OK : -ENOENT;
}
#endif

#ifdef CONFIG_NET_IPv6
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

#endif /* CONFIG_NET && CONFIG_NET_ROUTE  */
