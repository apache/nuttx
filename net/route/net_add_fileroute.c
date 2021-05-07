/****************************************************************************
 * net/route/net_add_fileroute.c
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
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/ip.h>

#include "route/fileroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_FILEROUTE) || defined(CONFIG_ROUTE_IPv6_FILEROUTE)

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

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_addroute_ipv4(in_addr_t target, in_addr_t netmask, in_addr_t router)
{
  struct net_route_ipv4_s route;
  struct file fshandle;
  ssize_t nwritten;
  int ret;

  /* Format the new routing table entry */

  net_ipv4addr_copy(route.target, target);
  net_ipv4addr_copy(route.netmask, netmask);
  net_ipv4addr_copy(route.router, router);
  net_ipv4_dumproute("New route", &route);

  /* Open the IPv4 routing table for append access */

  ret = net_openroute_ipv4(O_WRONLY | O_APPEND | O_CREAT, &fshandle);
  if (ret < 0)
    {
      nerr("ERROR: Could not open IPv4 routing table: %d\n", ret);
      return ret;
    }

  /* Then append the new entry to the end of the routing table */

  nwritten = net_writeroute_ipv4(&fshandle, &route);

  net_closeroute_ipv4(&fshandle);
  return nwritten >= 0 ? 0 : (int)nwritten;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_addroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask,
                      net_ipv6addr_t router)
{
  struct net_route_ipv6_s route;
  struct file fshandle;
  ssize_t nwritten;
  int ret;

  /* Format the new routing table entry */

  net_ipv6addr_copy(route.target, target);
  net_ipv6addr_copy(route.netmask, netmask);
  net_ipv6addr_copy(route.router, router);
  net_ipv6_dumproute("New route", &route);

  /* Open the IPv6 routing table for append access */

  ret = net_openroute_ipv6(O_WRONLY | O_APPEND | O_CREAT, &fshandle);
  if (ret < 0)
    {
      nerr("ERROR: Could not open IPv6 routing table: %d\n", ret);
      return ret;
    }

  /* Then append the new entry to the end of the routing table */

  nwritten = net_writeroute_ipv6(&fshandle, &route);

  net_closeroute_ipv6(&fshandle);
  return nwritten >= 0 ? 0 : (int)nwritten;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE */
