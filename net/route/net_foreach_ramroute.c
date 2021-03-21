/****************************************************************************
 * net/route/net_foreach_ramroute.c
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

#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "route/ramroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_foreachroute_ipv4 and net_foreachroute_ipv6
 *
 * Description:
 *   Traverse the routing table
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table.
 *   arg     - An arbitrary value that will be passed to the handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was search.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero, non-negative value.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
int net_foreachroute_ipv4(route_handler_ipv4_t handler, FAR void *arg)
{
  FAR struct net_route_ipv4_entry_s *route;
  FAR struct net_route_ipv4_entry_s *next;
  int ret = 0;

  /* Prevent concurrent access to the routing table */

  net_lock();

  /* Visit each entry in the routing table */

  for (route = g_ipv4_routes.head; ret == 0 && route != NULL; route = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the handler may delete this entry.
       */

      next = route->flink;
      ret  = handler(&route->entry, arg);
    }

  /* Unlock the network */

  net_unlock();
  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg)
{
  FAR struct net_route_ipv6_entry_s *route;
  FAR struct net_route_ipv6_entry_s *next;
  int ret = 0;

  /* Prevent concurrent access to the routing table */

  net_lock();

  /* Visit each entry in the routing table */

  for (route = g_ipv6_routes.head; ret == 0 && route != NULL; route = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the handler may delete this entry.
       */

      next = route->flink;
      ret  = handler(&route->entry, arg);
    }

  /* Unlock the network */

  net_unlock();
  return ret;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE */
