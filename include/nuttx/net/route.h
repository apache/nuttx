/****************************************************************************
 * include/nuttx/net/route.h
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

#ifndef __INCLUDE_NUTTX_NET_ROUTE_H
#define __INCLUDE_NUTTX_NET_ROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/uip/uip.h>

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
  bool         inuse;    /* TRUE: This entry contains a valid route */
  uint8_t      minor;    /* Ethernet device minor */
  uip_ipaddr_t target;   /* The destination network */
  uip_ipaddr_t netmask;  /* The network address mask */
  uip_ipaddr_t gateway;  /* Route packets via a gateway */
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

EXTERN struct net_route_s g_routes[CONFIG_NET_MAXROUTES];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: net_addroute
 *
 * Description:
 *   Add a new route to the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int net_addroute(uip_ipaddr_t target, uip_ipaddr_t netmask,
                 uip_ipaddr_t gateway, int devno);

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

int net_delroute(uip_ipaddr_t target, uip_ipaddr_t netmask);

/****************************************************************************
 * Function: net_findroute
 *
 * Description:
 *   Given an IP address, return a copy of the routing table contents
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int net_findroute(uip_ipaddr_t target, FAR struct net_route_s *route);

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
#endif /* __INCLUDE_NUTTX_NET_ROUTE_H */
