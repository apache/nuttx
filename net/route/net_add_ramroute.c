/****************************************************************************
 * net/route/net_add_ramroute.c
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

#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include <arch/irq.h>

#include "route/ramroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_addroute_ipv4 and net_addroute_ipv6
 *
 * Description:
 *   Add a new route to the routing table
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
int net_addroute_ipv4(in_addr_t target, in_addr_t netmask, in_addr_t router)
{
  FAR struct net_route_ipv4_s *route;

  /* Allocate a route entry */

  route = net_allocroute_ipv4();
  if (!route)
    {
      nerr("ERROR:  Failed to allocate a route\n");
      return -ENOMEM;
    }

  /* Format the new routing table entry */

  net_ipv4addr_copy(route->target, target);
  net_ipv4addr_copy(route->netmask, netmask);
  net_ipv4addr_copy(route->router, router);
  net_ipv4_dumproute("New route", route);

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the new entry to the table */

  ramroute_ipv4_addlast((FAR struct net_route_ipv4_entry_s *)route,
                        &g_ipv4_routes);
  net_unlock();
  return OK;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
int net_addroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask,
                      net_ipv6addr_t router)
{
  FAR struct net_route_ipv6_s *route;

  /* Allocate a route entry */

  route = net_allocroute_ipv6();
  if (!route)
    {
      nerr("ERROR:  Failed to allocate a route\n");
      return -ENOMEM;
    }

  /* Format the new routing table entry */

  net_ipv6addr_copy(route->target, target);
  net_ipv6addr_copy(route->netmask, netmask);
  net_ipv6addr_copy(route->router, router);
  net_ipv6_dumproute("New route", route);

  /* Get exclusive address to the networking data structures */

  net_lock();

  /* Then add the new entry to the table */

  ramroute_ipv6_addlast((FAR struct net_route_ipv6_entry_s *)route,
                        &g_ipv6_routes);
  net_unlock();
  return OK;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE */
