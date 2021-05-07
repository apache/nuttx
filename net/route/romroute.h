/****************************************************************************
 * net/route/romroute.h
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
