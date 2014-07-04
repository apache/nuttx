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

struct net_route_s
{
  FAR struct net_route_s *flink; /* Supports a singly linked list */
  net_ipaddr_t target;           /* The destination network */
  net_ipaddr_t netmask;          /* The network address mask */
  net_ipaddr_t router;           /* Route packets via this router */
};

/* Type of the call out function pointer provided to net_foreachroute() */

typedef int (*route_handler_t)(FAR struct net_route_s *route, FAR void *arg);

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

EXTERN sq_queue_t g_routes;

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

FAR struct net_route_s *net_allocroute(void);

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

void net_freeroute(FAR struct net_route_s *route);

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

int net_addroute(net_ipaddr_t target, net_ipaddr_t netmask,
                 net_ipaddr_t router);

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

int net_delroute(net_ipaddr_t target, net_ipaddr_t netmask);

/****************************************************************************
 * Function: net_router
 *
 * Description:
 *   Given an IP address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *
 * Parameters:
 *   target - An IP address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 *   NOTE:  For IPv6, router will be an array, for IPv4 it will be a scalar
 *   value.  Hence, the change in the function signature.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int net_router(net_ipaddr_t target, net_ipaddr_t router);
#else
int net_router(net_ipaddr_t target, FAR net_ipaddr_t *router);
#endif

/****************************************************************************
 * Function: netdev_router
 *
 * Description:
 *   Given an IP address on a external network, return the address of the
 *   router on a local network that can forward to the external network.
 *   This is similar to net_router().  However, the set of routers is
 *   constrained to those accessible by the specific device
 *
 * Parameters:
 *   dev    - We are committed to using this device.
 *   target - An IP address on a remote network to use in the lookup.
 *   router - The address of router on a local network that can forward our
 *     packets to the target.
 *
 *   NOTE:  For IPv6, router will be an array, for IPv4 it will be a scalar
 *   value.  Hence, the change in the function signature.
 *
 * Returned Value:
 *   None, a router address is always returned (which may just be, perhaps,
 *   device's default router address)
 *
 ****************************************************************************/

struct net_driver_s;

#ifdef CONFIG_NET_IPv6
void netdev_router(FAR struct net_driver_s *dev, net_ipaddr_t target,
                   net_ipaddr_t router);
#else
void netdev_router(FAR struct net_driver_s *dev, net_ipaddr_t target,
                   FAR net_ipaddr_t *router);
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

int net_foreachroute(route_handler_t handler, FAR void *arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ROUTE */
#endif /* __NET_ROUTE_ROUTE_H */
