/****************************************************************************
 * net/route/romroute.h
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

#ifndef __NET_ROUTE_ROMROUTE_H
#define __NET_ROUTE_ROMROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_ROMROUTE) || defined(CONFIG_ROUTE_IPv6_ROMROUTE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These are the routing tables.  These must be provided by board-specific
 * logic.
 */

#if defined(CONFIG_ROUTE_IPv4_ROMROUTE)
/* The in-memory routing tables are represented as a simple array. */

extern struct net_route_ipv4_s g_ipv4_routes[];
extern const unsigned int g_ipv4_nroutes;
#endif

#if defined(CONFIG_ROUTE_IPv6_ROMROUTE)
/* The in-memory routing tables are represented as a simple array. */

extern struct net_route_ipv6_s g_ipv6_routes[];
extern const unsigned int g_ipv6_nroutes;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_ROUTE_IPv4_ROMROUTE || CONFIG_ROUTE_IPv6_ROMROUTE */
#endif /* __NET_ROUTE_ROMROUTE_H */
