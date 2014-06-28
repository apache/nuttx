/****************************************************************************
 * net/route/netdev_router.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct route_devmatch_s
{
  FAR struct net_driver_s *dev; /* The route must use this device */
  uip_ipaddr_t target;   /* The target IP address on an external network to match */
  uip_ipaddr_t router;   /* The IP address of the router on one of our networks*/
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_devmatch
 *
 * Description:
 *   Return 1 if the route is available on the device's network.
 *
 * Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

static int net_devmatch(FAR struct net_route_s *route, FAR void *arg)
{
  FAR struct route_devmatch_s *match = (FAR struct route_devmatch_s *)arg;
  FAR struct net_driver_s *dev = match->dev;

  /* To match, (1) the masked target addresses must be the same, and (2) the
   * router address must like on the network provided by the device.
   *
   * In the event of multiple matches, only the first is returned.  There
   * not (yet) any concept for the precedence of networks.
   */

  if (uip_ipaddr_maskcmp(route->target, match->target, route->netmask) &&
      uip_ipaddr_maskcmp(route->router, dev->d_ipaddr, dev->d_netmask))
    {
      /* They match.. Copy the router address */

      uip_ipaddr_copy(match->router, route->router);
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void netdev_router(FAR struct net_driver_s *dev, uip_ipaddr_t target,
                   uip_ipaddr_t router)
#else
void netdev_router(FAR struct net_driver_s *dev, uip_ipaddr_t target,
                   FAR uip_ipaddr_t *router)
#endif
{
  struct route_devmatch_s match;
  int ret;

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_devmatch_s));
  match.dev = dev;
  uip_ipaddr_copy(match.target, target);

  /* Find an router entry with the routing table that can forward to this
   * address using this device.
   */

  ret = net_foreachroute(net_devmatch, &match);
  if (ret > 0)
    {
      /* We found a route.  Return the router address. */

#ifdef CONFIG_NET_IPv6
      uip_ipaddr_copy(router, match.target);
#else
      uip_ipaddr_copy(*router, match.target);
#endif
      ret = OK;
    }
  else
    {
      /* There isn't a matching route.. fallback and use the default router
       * of the device.
       */

#ifdef CONFIG_NET_IPv6
      uip_ipaddr_copy(router, dev->d_draddr);
#else
      uip_ipaddr_copy(*router, dev->d_draddr);
#endif
    }
}

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
