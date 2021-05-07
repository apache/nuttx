/****************************************************************************
 * net/route/net_alloc_ramroute.c
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

/* These are lists of free routing table entries */

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
static struct net_route_ipv4_queue_s g_free_ipv4routes;
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
static struct net_route_ipv6_queue_s g_free_ipv6routes;
#endif

/* These are arrays of pre-allocated network routes */

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
static struct net_route_ipv4_entry_s
  g_prealloc_ipv4routes[CONFIG_ROUTE_MAX_IPv4_RAMROUTES];
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
static struct net_route_ipv6_entry_s
  g_prealloc_ipv6routes[CONFIG_ROUTE_MAX_IPv6_RAMROUTES];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_ramroute
 *
 * Description:
 *   Initialize the in-memory, RAM routing table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_init_ramroute(void)
{
  int i;

  /* Initialize the routing table and the free list */

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
  ramroute_init(&g_ipv4_routes);
  ramroute_init(&g_free_ipv4routes);

  /* Add all of the pre-allocated routing table entries to a free list */

  for (i = 0; i < CONFIG_ROUTE_MAX_IPv4_RAMROUTES; i++)
    {
      ramroute_ipv4_addlast(&g_prealloc_ipv4routes[i], &g_free_ipv4routes);
    }
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
  ramroute_init(&g_ipv6_routes);
  ramroute_init(&g_free_ipv6routes);

  /* Add all of the pre-allocated routing table entries to a free list */

  for (i = 0; i < CONFIG_ROUTE_MAX_IPv6_RAMROUTES; i++)
    {
      ramroute_ipv6_addlast(&g_prealloc_ipv6routes[i], &g_free_ipv6routes);
    }
#endif
}

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

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the remove the first entry from the table */

  route = ramroute_ipv4_remfirst(&g_free_ipv4routes);

  net_unlock();
  return &route->entry;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void)
{
  FAR struct net_route_ipv6_entry_s *route;

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the remove the first entry from the table */

  route = ramroute_ipv6_remfirst(&g_free_ipv6routes);

  net_unlock();
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

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the new entry to the table */

  ramroute_ipv4_addlast((FAR struct net_route_ipv4_entry_s *)route,
                        &g_free_ipv4routes);
  net_unlock();
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route)
{
  DEBUGASSERT(route);

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the new entry to the table */

  ramroute_ipv6_addlast((FAR struct net_route_ipv6_entry_s *)route,
                        &g_free_ipv6routes);
  net_unlock();
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE */
