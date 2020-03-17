/****************************************************************************
 * net/route/cacheroute.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __NET_ROUTE_CACHEROUTE_H
#define __NET_ROUTE_CACHEROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_CACHEROUTE) || defined(CONFIG_ROUTE_IPv6_CACHEROUTE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_cacheroute
 *
 * Description:
 *   Initialize the in-memory, routing table cache
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

void net_init_cacheroute(void);

/****************************************************************************
 * Name: net_addcache_ipv4 and net_addcache_ipv6
 *
 * Description:
 *   Add one route to the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
int net_addcache_ipv4(FAR struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
int net_addcache_ipv6(FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: net_foreachcache_ipv4/net_foreachcache_ipv6
 *
 * Description:
 *   Traverse the routing table cache
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table cache.
 *   arg     - An arbitrary value that will be passed tot he handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was searched.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero value.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
int net_foreachcache_ipv4(route_handler_ipv4_t handler, FAR void *arg);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
int net_foreachcache_ipv6(route_handler_ipv6_t handler, FAR void *arg);
#endif

/****************************************************************************
 * Name: net_flushcache_ipv4 and net_flushcache_ipv6
 *
 * Description:
 *   Flush the content of the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
void net_flushcache_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
void net_flushcache_ipv6(void);
#endif

#endif /* CONFIG_ROUTE_IPv4_CACHEROUTE || CONFIG_ROUTE_IPv6_CACHEROUTE */
#endif /* __NET_ROUTE_CACHEROUTE_H */
