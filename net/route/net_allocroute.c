/****************************************************************************
 * net/route/net_allocroute.c
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
#include <assert.h>

#include <nuttx/net/net.h>
#include <arch/irq.h>

#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the routing table */

#ifdef CONFIG_NET_IPv4
sq_queue_t g_routes;
#endif

#ifdef CONFIG_NET_IPv6
sq_queue_t g_routes_ipv6;
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a list of free routing table entries */

#ifdef CONFIG_NET_IPv4
static sq_queue_t g_freeroutes;
#endif

#ifdef CONFIG_NET_IPv6
static sq_queue_t g_freeroutes_ipv6;
#endif

/* This is an array of pre-allocated network routes */

#ifdef CONFIG_NET_IPv4
static struct net_route_s g_preallocroutes[CONFIG_NET_MAXROUTES];
#endif

#ifdef CONFIG_NET_IPv6
static struct net_route_ipv6_s g_preallocroutes_ipv6[CONFIG_NET_MAXROUTES];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_initroute
 *
 * Description:
 *   Initialize to the routing table
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_initroute(void)
{
  int i;

  /* Initialize the routing table and the free list */

#ifdef CONFIG_NET_IPv4
  sq_init(&g_routes);
  sq_init(&g_freeroutes);

  /* All all of the pre-allocated routing table entries to a free list */

  for (i = 0; i < CONFIG_NET_MAXROUTES; i++)
    {
      sq_addlast((FAR sq_entry_t *)&g_preallocroutes[i],
                 (FAR sq_queue_t *)&g_freeroutes);
    }
#endif

#ifdef CONFIG_NET_IPv6
  sq_init(&g_routes_ipv6);
  sq_init(&g_freeroutes_ipv6);

  /* All all of the pre-allocated routing table entries to a free list */

  for (i = 0; i < CONFIG_NET_MAXROUTES; i++)
    {
      sq_addlast((FAR sq_entry_t *)&g_preallocroutes_ipv6[i],
                 (FAR sq_queue_t *)&g_freeroutes_ipv6);
    }
#endif
}

/****************************************************************************
 * Function: net_allocroute
 *
 * Description:
 *   Allocate one route by removing it from the free list
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the newly allocated route table entry is
 *   returned; NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_route_s *net_allocroute(void)
{
  FAR struct net_route_s *route;
  net_lock_t save;

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  route = (FAR struct net_route_s *)
    sq_remfirst((FAR sq_queue_t *)&g_freeroutes);

  net_unlock(save);
  return route;
}
#endif

#ifdef CONFIG_NET_IPv6
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void)
{
  FAR struct net_route_ipv6_s *route;
  net_lock_t save;

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  route = (FAR struct net_route_ipv6_s *)
    sq_remfirst((FAR sq_queue_t *)&g_freeroutes_ipv6);

  net_unlock(save);
  return route;
}
#endif

/****************************************************************************
 * Function: net_allocroute
 *
 * Description:
 *   Free one route by adding it from the free list
 *
 * Parameters:
 *   route - The route to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void net_freeroute(FAR struct net_route_s *route)
{
  net_lock_t save;

  DEBUGASSERT(route);

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  sq_addlast((FAR sq_entry_t *)route, (FAR sq_queue_t *)&g_freeroutes);
  net_unlock(save);
}
#endif

#ifdef CONFIG_NET_IPv6
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route)
{
  net_lock_t save;

  DEBUGASSERT(route);

  /* Get exclusive address to the networking data structures */

  save = net_lock();

  /* Then add the new entry to the table */

  sq_addlast((FAR sq_entry_t *)route, (FAR sq_queue_t *)&g_freeroutes_ipv6);
  net_unlock(save);
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
