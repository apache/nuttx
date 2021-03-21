/****************************************************************************
 * net/route/route.h
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

#ifndef __NET_ROUTE_ROUTE_H
#define __NET_ROUTE_ROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <net/if.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_ROUTE

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes one entry in the routing table */

#ifdef CONFIG_NET_IPv4
struct net_route_ipv4_s
{
  in_addr_t target;          /* The destination network */
  in_addr_t netmask;         /* The network address mask */
  in_addr_t router;          /* Route packets via this router */
};

/* Type of the call out function pointer provided to
 * net_foreachroute_ipv4()
 */

typedef int (*route_handler_ipv4_t)(FAR struct net_route_ipv4_s *route,
                               FAR void *arg);
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
struct net_route_ipv6_s
{
  net_ipv6addr_t target;     /* The destination network */
  net_ipv6addr_t netmask;    /* The network address mask */
  net_ipv6addr_t router;     /* Route packets via this router */
};

/* Type of the call out function pointer provided to
 * net_foreachroute_ipv6()
 */

typedef int (*route_handler_ipv6_t)(FAR struct net_route_ipv6_s *route,
                                    FAR void *arg);
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

void net_init_route(void);

/****************************************************************************
 * Name: net_addroute_ipv4 and net_addroute_ipv6
 *
 * Description:
 *   Add a new route to the routing table
 *
 * Input Parameters:
 *   target   - The destination IP address on the destination network
 *   netmask  - The mask defining the destination sub-net
 *   router   - The IP address on one of our networks that provides the
 *              router to the external network
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_addroute_ipv4(in_addr_t target, in_addr_t netmask,
                      in_addr_t router);
#endif

#ifdef CONFIG_NET_IPv6
int net_addroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask,
                 net_ipv6addr_t router);
#endif

/****************************************************************************
 * Name: net_delroute_ipv4 and net_delroute_ipv6
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_delroute_ipv4(in_addr_t target, in_addr_t netmask);
#endif

#ifdef CONFIG_NET_IPv6
int net_delroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask);
#endif

/****************************************************************************
 * Name: net_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Input Parameters:
 *   target - An IPv4 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_ipv4_router(in_addr_t target, FAR in_addr_t *router);
#endif

/****************************************************************************
 * Name: net_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Input Parameters:
 *   target - An IPv6 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int net_ipv6_router(const net_ipv6addr_t target, net_ipv6addr_t router);
#endif

/****************************************************************************
 * Name: netdev_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv4_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Input Parameters:
 *   dev    - We are committed to using this device.
 *   target - An IPv4 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   A router address is always returned (which may just be, perhaps,
 *   device's default router address)
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
struct net_driver_s;
void netdev_ipv4_router(FAR struct net_driver_s *dev, in_addr_t target,
                        FAR in_addr_t *router);
#endif

/****************************************************************************
 * Name: netdev_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv6_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Input Parameters:
 *   dev    - We are committed to using this device.
 *   target - An IPv6 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   A router address is always returned (which may just be, perhaps,
 *   device's default router address)
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
struct net_driver_s;
void netdev_ipv6_router(FAR struct net_driver_s *dev,
                        FAR const net_ipv6addr_t target,
                        FAR net_ipv6addr_t router);
#endif

/****************************************************************************
 * Name: net_foreachroute_ipv4/net_foreachroute_ipv6
 *
 * Description:
 *   Traverse the routing table
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table.
 *   arg     - An arbitrary value that will be passed to the handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was searched.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_foreachroute_ipv4(route_handler_ipv4_t handler, FAR void *arg);
#endif

#ifdef CONFIG_NET_IPv6
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg);
#endif

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

#ifdef CONFIG_DEBUG_NET_INFO
#ifdef CONFIG_NET_IPv4
void net_ipv4_dumproute(FAR const char *msg,
                        FAR struct net_route_ipv4_s *route);
#else
#  define net_ipv4_dumproute(m,r)
#endif

#ifdef CONFIG_NET_IPv6
void net_ipv6_dumproute(FAR const char *msg,
                        FAR struct net_route_ipv6_s *route);
#else
#  define net_ipv6_dumproute(m,r)
#endif

#else
#  define net_ipv4_dumproute(m,r)
#  define net_ipv6_dumproute(m,r)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ROUTE */
#endif /* __NET_ROUTE_ROUTE_H */
