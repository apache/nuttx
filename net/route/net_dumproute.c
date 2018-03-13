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
