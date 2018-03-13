/****************************************************************************
 * net/route/net_foreach_romroute.c
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
 *   arg     - An arbitrary value that will be passed tot he handler.
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
