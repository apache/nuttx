/****************************************************************************
 * net/route/net_dumproute.c
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

#include <debug.h>
#include <arpa/inet.h>

#include "route/route.h"

#if defined(CONFIG_NET_ROUTE) && defined(CONFIG_DEBUG_NET_INFO)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ipv4_dumproute and net_ipv6_dumproute
 *
 * Description:
 *   Dump a routing table entry
 *
 * Input Parameters:
 *   route - The entry to be dumped
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void net_ipv4_dumproute(FAR const char *msg,
                        FAR struct net_route_ipv4_s *route)
{
  ninfo("%s:\n", msg);
  ninfo("  target=%08lx netmask=%08lx router=%08lx\n",
        htonl(route->target), htonl(route->netmask), htonl(route->router));
}
#endif

#ifdef CONFIG_NET_IPv6
void net_ipv6_dumproute(FAR const char *msg,
                        FAR struct net_route_ipv6_s *route)
{
  ninfo("%s:\n", msg);
  ninfo("  target:  %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        htons(route->target[0]),  htons(route->target[1]),
        htons(route->target[2]),  htons(route->target[3]),
        htons(route->target[4]),  htons(route->target[5]),
        htons(route->target[6]),  htons(route->target[7]));
  ninfo("  netmask: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        htons(route->netmask[0]), htons(route->netmask[1]),
        htons(route->netmask[2]), htons(route->netmask[3]),
        htons(route->netmask[4]), htons(route->netmask[5]),
        htons(route->netmask[6]), htons(route->netmask[7]));
  ninfo("  router:  %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        htons(route->router[0]),  htons(route->router[1]),
        htons(route->router[2]),  htons(route->router[3]),
        htons(route->router[4]),  htons(route->router[5]),
        htons(route->router[6]),  htons(route->router[7]));
}
#endif

#endif /* CONFIG_NET_ROUTE && CONFIG_DEBUG_NET_INFO */
