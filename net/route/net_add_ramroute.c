/****************************************************************************
 * net/route/net_add_ramroute.c
 *
 *   Copyright (C) 2013, 2015, 2017 Gregory Nutt. All rights reserved.
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
