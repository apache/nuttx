/****************************************************************************
 * net/route/net_foreachroute.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_foreachroute
 *
 * Description:
 *   Traverse the route table
 *
 * Parameters:
 *
 * Returned Value:
 *   0 if in use; 1 if avaialble and the new entry was added
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_foreachroute(route_handler_t handler, FAR void *arg)
{
  FAR struct net_route_s *route;
  FAR struct net_route_s *next;
  net_lock_t save;
  int ret = 0;

  /* Prevent concurrent access to the routing table */

  save = net_lock();

  /* Visit each entry in the routing table */

  for (route = (FAR struct net_route_s *)g_routes.head; route; route = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the hanlder may delete this entry.
       */

      next = route->flink;
      ret = handler(route, arg);
    }

  /* Unlock uIP */

  net_unlock(save);
  return ret;
}
#endif

#ifdef CONFIG_NET_IPv6
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg)
{
  FAR struct net_route_ipv6_s *route;
  FAR struct net_route_ipv6_s *next;
  net_lock_t save;
  int ret = 0;

  /* Prevent concurrent access to the routing table */

  save = net_lock();

  /* Visit each entry in the routing table */

  for (route = (FAR struct net_route_s *)g_routes_ipv6.head; route; route = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the hanlder may delete this entry.
       */

      next = route->flink;
      ret = handler(route, arg);
    }

  /* Unlock uIP */

  net_unlock(save);
  return ret;
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
