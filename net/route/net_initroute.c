/****************************************************************************
 * net/route/net_initroute.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "route/ramroute.h"
#include "route/cacheroute.h"
#include "route/route.h"

#ifdef CONFIG_NET_ROUTE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_route
 *
 * Description:
 *   Initialize to the routing table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_init_route(void)
{
#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)
  net_init_ramroute();
#endif

#if defined(CONFIG_ROUTE_IPv4_CACHEROUTE) || defined(CONFIG_ROUTE_IPv6_CACHEROUTE)
  net_init_cacheroute();
#endif
}

#endif /* CONFIG_NET_ROUTE */
