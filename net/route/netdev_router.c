/****************************************************************************
 * net/route/netdev_router.c
 *
 *   Copyright (C) 2013-2015 Gregory Nutt. All rights reserved.
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
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "route/route.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
struct route_ipv4_devmatch_s
{
  FAR struct net_driver_s *dev; /* The route must use this device */
  in_addr_t target;             /* Target IPv4 address on an external network to match */
  in_addr_t router;             /* IPv6 address of the router on one of our networks*/
};
#endif

#ifdef CONFIG_NET_IPv6
struct route_ipv6_devmatch_s
{
  FAR struct net_driver_s *dev; /* The route must use this device */
  net_ipv6addr_t target;        /* Target IPv4 address on an external network to match */
  net_ipv6addr_t router;        /* IPv6 address of the router on one of our networks*/
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_ipv4_devmatch
 *
 * Description:
 *   Return 1 if the IPv4 route is available on the device's network.
 *
 * Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int net_ipv4_devmatch(FAR struct net_route_s *route, FAR void *arg)
{
  FAR struct route_ipv4_devmatch_s *match = (FAR struct route_ipv4_devmatch_s *)arg;
  FAR struct net_driver_s *dev = match->dev;

  /* To match, (1) the masked target addresses must be the same, and (2) the
   * router address must like on the network provided by the device.
   *
   * In the event of multiple matches, only the first is returned.  There
   * not (yet) any concept for the precedence of networks.
   */

  if (net_ipv4addr_maskcmp(route->target, match->target, route->netmask) &&
      net_ipv4addr_maskcmp(route->router, dev->d_ipaddr, dev->d_netmask))
    {
      /* They match.. Copy the router address */

      net_ipv4addr_copy(match->router, route->router);
      return 1;
    }

  return 0;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Function: net_ipv6_devmatch
 *
 * Description:
 *   Return 1 if the IPv6 route is available on the device's network.
 *
 * Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int net_ipv6_devmatch(FAR struct net_route_ipv6_s *route, FAR void *arg)
{
  FAR struct route_ipv6_devmatch_s *match = (FAR struct route_ipv6_devmatch_s *)arg;
  FAR struct net_driver_s *dev = match->dev;

  /* To match, (1) the masked target addresses must be the same, and (2) the
   * router address must like on the network provided by the device.
   *
   * In the event of multiple matches, only the first is returned.  There
   * not (yet) any concept for the precedence of networks.
   */

  if (net_ipv6addr_maskcmp(route->target, match->target, route->netmask) &&
      net_ipv6addr_maskcmp(route->router, dev->d_ipv6addr, dev->d_ipv6netmask))
    {
      /* They match.. Copy the router address */

      net_ipv6addr_copy(match->router, route->router);
      return 1;
    }

  return 0;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
void netdev_ipv4_router(FAR struct net_driver_s *dev, in_addr_t target,
                        FAR in_addr_t *router)
{
  struct route_ipv4_devmatch_s match;
  int ret;

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv4_devmatch_s));
  match.dev = dev;
  net_ipv4addr_copy(match.target, target);

  /* Find an router entry with the routing table that can forward to this
   * address using this device.
   */

  ret = net_foreachroute(net_ipv4_devmatch, &match);
  if (ret > 0)
    {
      /* We found a route.  Return the router address. */

      net_ipv4addr_copy(*router, match.target);
    }
  else
    {
      /* There isn't a matching route.. fallback and use the default router
       * of the device.
       */

      net_ipv4addr_copy(*router, dev->d_draddr);
    }
}
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
void netdev_ipv6_router(FAR struct net_driver_s *dev,
                        FAR const net_ipv6addr_t target,
                        FAR net_ipv6addr_t router)
{
  struct route_ipv6_devmatch_s match;
  int ret;

  /* Set up the comparison structure */

  memset(&match, 0, sizeof(struct route_ipv6_devmatch_s));
  match.dev = dev;
  net_ipv6addr_copy(match.target, target);

  /* Find an router entry with the routing table that can forward to this
   * address using this device.
   */

  ret = net_foreachroute_ipv6(net_ipv6_devmatch, &match);
  if (ret > 0)
    {
      /* We found a route.  Return the router address. */

      net_ipv6addr_copy(router, match.target);
    }
  else
    {
      /* There isn't a matching route.. fallback and use the default router
       * of the device.
       */

      net_ipv6addr_copy(router, dev->d_ipv6draddr);
    }
}
#endif

#endif /* CONFIG_NET && CONFIG_NET_ROUTE */
