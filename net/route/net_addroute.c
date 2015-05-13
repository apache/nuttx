/****************************************************************************
 * net/route/net_addroute.c
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
#include <queue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include <arch/irq.h>

#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_addroute
 *
 * Description:
 *   Add a new route to the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_addroute(in_addr_t target, in_addr_t netmask, in_addr_t router)
{
  FAR struct net_route_s *route;
  net_lock_t save;

  /* Allocate a route entry */

  route = net_allocroute();
  if (!route)
    {
      ndbg("ERROR:  Failed to allocate a route\n");
      return -ENOMEM;
    }

  /* Format the new route table entry */

  net_ipv4addr_copy(route->target, target);
  net_ipv4addr_copy(route->netmask, netmask);
  net_ipv4addr_copy(route->router, router);

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  sq_addlast((FAR sq_entry_t *)route, (FAR sq_queue_t *)&g_routes);
  net_unlock(save);
  return OK;
}
#endif

#ifdef CONFIG_NET_IPv6
int net_addroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask, net_ipv6addr_t router)
{
  FAR struct net_route_ipv6_s *route;
  net_lock_t save;

  /* Allocate a route entry */

  route = net_allocroute_ipv6();
  if (!route)
    {
      ndbg("ERROR:  Failed to allocate a route\n");
      return -ENOMEM;
    }

  /* Format the new route table entry */

  net_ipv6addr_copy(route->target, target);
  net_ipv6addr_copy(route->netmask, netmask);
  net_ipv6addr_copy(route->router, router);

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  sq_addlast((FAR sq_entry_t *)route, (FAR sq_queue_t *)&g_routes_ipv6);
  net_unlock(save);
  return OK;
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
