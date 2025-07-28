/****************************************************************************
 * net/route/net_alloc_ramroute.c
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
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <arch/irq.h>

#include "utils/utils.h"
#include "route/ramroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These are the routing tables.  The in-memory routing tables are
 * represented as singly linked lists.
 */

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
FAR struct net_route_ipv4_queue_s g_ipv4_routes;
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_queue_s g_ipv6_routes;
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all routing table entries. */

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
NET_BUFPOOL_DECLARE(g_ipv4routes, sizeof(struct net_route_ipv4_entry_s),
                    CONFIG_ROUTE_MAX_IPv4_RAMROUTES, 0, 0);
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
NET_BUFPOOL_DECLARE(g_ipv6routes, sizeof(struct net_route_ipv6_entry_s),
                    CONFIG_ROUTE_MAX_IPv6_RAMROUTES, 0, 0);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_allocroute_ipv4 and net_allocroute_ipv6
 *
 * Description:
 *   Allocate one route by removing it from the free list
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the newly allocated routing table entry is
 *   returned; NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
FAR struct net_route_ipv4_s *net_allocroute_ipv4(void)
{
  FAR struct net_route_ipv4_entry_s *route;

  /* Get exclusive address to the networking data structures and
   * then remove the first entry from the g_ipv4routes pool
   */

  route = NET_BUFPOOL_TRYALLOC(g_ipv4routes);
  if (!route)
    {
      return NULL;
    }

  return &route->entry;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void)
{
  FAR struct net_route_ipv6_entry_s *route;

  /* Get exclusive address to the networking data structures and
   * then remove the first entry from the g_ipv6routes pool
   */

  route = NET_BUFPOOL_TRYALLOC(g_ipv6routes);
  if (!route)
    {
      return NULL;
    }

  return &route->entry;
}
#endif

/****************************************************************************
 * Name: net_freeroute_ipv4 and net_freeroute_ipv6
 *
 * Description:
 *   Free one route by adding it from the free list
 *
 * Input Parameters:
 *   route - The route to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
void net_freeroute_ipv4(FAR struct net_route_ipv4_s *route)
{
  DEBUGASSERT(route);

  /* Add the new entry to the g_ipv4routes pool */

  NET_BUFPOOL_FREE(g_ipv4routes, (FAR struct net_route_ipv4_entry_s *)route);
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route)
{
  DEBUGASSERT(route);

  /* Add the new entry to the g_ipv6routes pool */

  NET_BUFPOOL_FREE(g_ipv6routes, (FAR struct net_route_ipv6_entry_s *)route);
}
#endif

/****************************************************************************
 * Name: net_lockroute_ipv4 and net_unlockroute_ipv4
 *
 * Description:
 *   Lock and unlock the g_ipv4routes pool
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
void net_lockroute_ipv4(void)
{
  NET_BUFPOOL_LOCK(g_ipv4routes);
}

void net_unlockroute_ipv4(void)
{
  NET_BUFPOOL_UNLOCK(g_ipv4routes);
}
#endif

/****************************************************************************
 * Name: net_lockroute_ipv6 and net_unlockroute_ipv6
 *
 * Description:
 *   Lock and unlock the g_ipv6routes pool
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void net_lockroute_ipv6(void)
{
  NET_BUFPOOL_LOCK(g_ipv6routes);
}

void net_unlockroute_ipv6(void)
{
  NET_BUFPOOL_UNLOCK(g_ipv6routes);
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE */
