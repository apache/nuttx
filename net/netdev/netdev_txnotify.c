/****************************************************************************
 * net/netdev/netdev_txnotify.c
 *
 *   Copyright (C) 2007-2009, 2012, 2014 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ipv4_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Input Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void netdev_ipv4_txnotify(in_addr_t lipaddr, in_addr_t ripaddr)
{
  FAR struct net_driver_s *dev;

  /* Find the device driver that serves the subnet of the remote address */

  dev = netdev_findby_ipv4addr(lipaddr, ripaddr);
  if (dev && dev->d_txavail)
    {
      /* Notify the device driver that new TX data is available. */

      (void)dev->d_txavail(dev);
    }
}
#endif /* CONFIG_NET_IPv4 */


/****************************************************************************
 * Name: netdev_ipv6_txnotify
 *
 * Description:
 *   Notify the device driver that forwards the IPv4 address that new TX
 *   data is available.
 *
 * Input Parameters:
 *   lipaddr - The local address bound to the socket
 *   ripaddr - The remote address to send the data
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void netdev_ipv6_txnotify(FAR const net_ipv6addr_t lipaddr,
                          FAR const net_ipv6addr_t ripaddr)
{
  FAR struct net_driver_s *dev;

  /* Find the device driver that serves the subnet of the remote address */

  dev = netdev_findby_ipv6addr(lipaddr, ripaddr);
  if (dev && dev->d_txavail)
    {
      /* Notify the device driver that new TX data is available. */

      (void)dev->d_txavail(dev);
    }
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: netdev_txnotify_dev
 *
 * Description:
 *   Notify the device driver that new TX data is available.  This variant
 *   would be called when the upper level logic already understands how the
 *   packet will be routed.
 *
 * Input Parameters:
 *   dev - The network device driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void netdev_txnotify_dev(FAR struct net_driver_s *dev)
{
  if (dev != NULL && dev->d_txavail != NULL)
    {
      /* Notify the device driver that new TX data is available. */

      (void)dev->d_txavail(dev);
    }
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
