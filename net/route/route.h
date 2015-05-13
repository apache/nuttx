/****************************************************************************
 * net/route/route.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __NET_ROUTE_ROUTE_H
#define __NET_ROUTE_ROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <queue.h>

#include <net/if.h>

#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_ROUTE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_NET_MAXROUTES
#  define CONFIG_NET_MAXROUTES 4
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure describes one entry in the routing table */

#ifdef CONFIG_NET_IPv4
struct net_route_s
{
  FAR struct net_route_s *flink; /* Supports a singly linked list */
  in_addr_t target;              /* The destination network */
  in_addr_t netmask;             /* The network address mask */
  in_addr_t router;              /* Route packets via this router */
};

/* Type of the call out function pointer provided to net_foreachroute() */

typedef int (*route_handler_t)(FAR struct net_route_s *route, FAR void *arg);
#endif

#ifdef CONFIG_NET_IPv6
struct net_route_ipv6_s
{
  FAR struct net_route_ipv6_s *flink; /* Supports a singly linked list */
  net_ipv6addr_t target;              /* The destination network */
  net_ipv6addr_t netmask;             /* The network address mask */
  net_ipv6addr_t router;              /* Route packets via this router */
};

/* Type of the call out function pointer provided to net_foreachroute() */

typedef int (*route_handler_ipv6_t)(FAR struct net_route_ipv6_s *route, FAR void *arg);
#endif

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

/* This is the routing table */
#ifdef CONFIG_NET_IPv4
EXTERN sq_queue_t g_routes;
#endif

#ifdef CONFIG_NET_IPv6
EXTERN sq_queue_t g_routes_ipv6;
#endif

/****************************************************************************
 * Public Function Prototypes
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
 ****************************************************************************/

void net_initroute(void);

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
FAR struct net_route_s *net_allocroute(void);
#endif

#ifdef CONFIG_NET_IPv6
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void);
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
void net_freeroute(FAR struct net_route_s *route);
#endif

#ifdef CONFIG_NET_IPv6
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Function: net_addroute
 *
 * Description:
 *   Add a new route to the routing table
 *
 * Parameters:
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
int net_addroute(in_addr_t target, in_addr_t netmask,
                 in_addr_t router);
#endif

#ifdef CONFIG_NET_IPv6
int net_addroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask,
                 net_ipv6addr_t router);
#endif

/****************************************************************************
 * Function: net_delroute
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_delroute(in_addr_t target, in_addr_t netmask);
#endif

#ifdef CONFIG_NET_IPv6
int net_delroute_ipv6(net_ipv6addr_t target, net_ipv6addr_t netmask);
#endif

/****************************************************************************
 * Function: net_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Parameters:
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
 * Function: net_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Parameters:
 *   target - An IPv6 address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int net_ipv6_router(net_ipv6addr_t target, net_ipv6addr_t router);
#endif

/****************************************************************************
 * Function: netdev_ipv4_router
 *
 * Description:
 *   Given an IPv4 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv4_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Parameters:
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
 * Function: netdev_ipv6_router
 *
 * Description:
 *   Given an IPv6 address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_ipv6_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Parameters:
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
 * Function: net_foreachroute
 *
 * Description:
 *   Traverse the route table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_foreachroute(route_handler_t handler, FAR void *arg);
#endif

#ifdef CONFIG_NET_IPv6
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ROUTE */
#endif /* __NET_ROUTE_ROUTE_H */
