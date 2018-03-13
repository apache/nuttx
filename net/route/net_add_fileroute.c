/****************************************************************************
 * net/route/net_add_fileroute.c
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
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
//#include <nuttx/net/net.h>
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

  (void)net_closeroute_ipv4(&fshandle);
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

  (void)net_closeroute_ipv6(&fshandle);
  return nwritten >= 0 ? 0 : (int)nwritten;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE */
