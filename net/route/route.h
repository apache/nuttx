/****************************************************************************
 * net/route/route.h
 *
 *   Copyright (C) 2013-2014, 2017 Gregory Nutt. All rights reserved.
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
struct net_route_ipv4_s
{
  FAR struct net_route_ipv4_s *flink; /* Supports a singly linked list */
  in_addr_t target;                   /* The destination network */
  in_addr_t netmask;                  /* The network address mask */
  in_addr_t router;                   /* Route packets via this router */
};

/* Type of the call out function pointer provided to net_foreachroute_ipv4() */

typedef int (*route_handler_t)(FAR struct net_route_ipv4_s *route,
                               FAR void *arg);
#endif

#ifdef CONFIG_NET_IPv6
struct net_route_ipv6_s
{
  FAR struct net_route_ipv6_s *flink; /* Supports a singly linked list */
  net_ipv6addr_t target;              /* The destination network */
  net_ipv6addr_t netmask;             /* The network address mask */
  net_ipv6addr_t router;              /* Route packets via this router */
};

/* Type of the call out function pointer provided to net_foreachroute_ipv6() */

typedef int (*route_handler_ipv6_t)(FAR struct net_route_ipv6_s *route,
                                    FAR void *arg);
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
EXTERN sq_queue_t g_ipv4_routes;
#endif

#ifdef CONFIG_NET_IPv6
EXTERN sq_queue_t g_ipv6_routes;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_initroute
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
 * Name: net_allocroute_ipv4 and net_allocroute_ipv6
 *
 * Description:
 *   Allocate one route by removing it from the free list
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the newly allocated routing table entry is
 *   returned; NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_route_ipv4_s *net_allocroute_ipv4(void);
#endif

#ifdef CONFIG_NET_IPv6
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void);
#endif

/****************************************************************************
 * Name: net_freeroute_ipv4 and net_freeroute_ipv6
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
void net_freeroute_ipv4(FAR struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_NET_IPv6
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: net_addroute_ipv4 and net_addroute_ipv6
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
 * Parameters:
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
 * Name: net_ipv6_router
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
 * Name: netdev_ipv6_router
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
 * Name: net_foreachroute_ipv4
 *
 * Description:
 *   Traverse the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_foreachroute_ipv4(route_handler_t handler, FAR void *arg);
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
 * Parameters:
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
