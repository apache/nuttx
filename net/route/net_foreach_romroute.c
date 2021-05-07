/****************************************************************************
 * net/route/net_foreach_romroute.c
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

#include "route/romroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_ROMROUTE) || defined(CONFIG_ROUTE_IPv6_ROMROUTE)

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

#ifdef CONFIG_ROUTE_IPv4_ROMROUTE
int net_foreachroute_ipv4(route_handler_ipv4_t handler, FAR void *arg)
{
  int ret = 0;
  int i;

  /* Visit each entry in the routing table */

  for (i = 0; ret == 0 && i < g_ipv4_nroutes; i++)
    {
      /* Call the handler. */

      ret  = handler(&g_ipv4_routes[i], arg);
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_ROMROUTE
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg)
{
  int ret = 0;
  int i;

  /* Visit each entry in the routing table */

  for (i = 0; ret == 0 && i < g_ipv6_nroutes; i++)
    {
      /* Call the handler. */

      ret  = handler(&g_ipv6_routes[i], arg);
    }

  return ret;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_ROMROUTE || CONFIG_ROUTE_IPv6_ROMROUTE */
