/****************************************************************************
 * net/netdev_findbyaddr.c
 *
 *   Copyright (C) 2007-2009, 2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/uip/uip-arch.h>

#include "net_route.h"
#include "net_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_maskcmp
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_finddevice
 *
 * Description:
 *   Find a previously registered network device by matching a local address
 *   with the subnet served by the device.  Only "up" devices are considered
 *   (since a "down" device has no meaningful address).
 *
 * Parameters:
 *   addr - Pointer to the remote address of a connection
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

static FAR struct uip_driver_s *netdev_finddevice(const uip_ipaddr_t addr)
{
  struct uip_driver_s *dev;

  /* Examine each registered network device */

  netdev_semtake();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0)
        {
          /* Yes.. check for an address match (under the netmask) */

          if (uip_ipaddr_maskcmp(dev->d_ipaddr, addr, dev->d_netmask))
            {
              /* Its a match */

              netdev_semgive();
              return dev;
            }
        }
    }

  /* No device with the matching address found */

  netdev_semgive();
  return NULL;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_findbyaddr
 *
 * Description:
 *   Find a previously registered network device by matching an arbitrary
 *   IP address.
 *
 * Parameters:
 *   addr - Pointer to the remote address of a connection
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 * Assumptions:
 *  Called from normal user mode
 *
 ****************************************************************************/

FAR struct uip_driver_s *netdev_findbyaddr(const uip_ipaddr_t addr)
{
  struct uip_driver_s *dev;
#ifdef CONFIG_NET_ROUTE
  uip_ipaddr_t router;
  int ret;
#endif

  /* First, see if the address maps to the a local network */

  dev = netdev_finddevice(addr);
  if (dev)
    {
      return dev;
    }

  /* No.. The address lies on an external network */

#ifdef CONFIG_NET_ROUTE
  /* If we have a routing table, then perhaps we can find the the local
   * address of a router that can forward packets to the external network.
   */

#ifdef CONFIG_NET_IPv6
  ret = net_router(addr, router);
#else
  ret = net_router(addr, &router);
#endif
  if (ret >= 0)
    {
      /* Success... try to find the network device associated with the local
       * router address
       */

      dev = netdev_finddevice(router);
      if (dev)
        {
          return dev;
        }
    }
#endif /* CONFIG_NET_ROUTE */

  /* The above lookup will fail if the packet is being sent out of our
   * out subnet to a router and there is no routing information.
   *
   * However, if there is only a single, registered network interface, then
   * the decision is pretty easy.  Use that device and its default router
   * address.
   */

  netdev_semtake();
  if (g_netdevices && !g_netdevices->flink)
    {
      dev = g_netdevices;
    }
  netdev_semgive();

  /* If we will did not find the network device, then we might as well fail
   * because we are not configured properly to determine the route to the
   * target.
   */

  return dev;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
