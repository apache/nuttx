/****************************************************************************
 * net/tcp/tcp_finddev.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <string.h>
#include <errno.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_find_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The locally bound address, laddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int tcp_find_ipv4_device(FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* Return success without using device notification if the locally bound
   * address is INADDR_ANY.  In this case, there may be multiple devices
   * that can provide data so the exceptional events from any particular
   * device are not important.
   */

  if (net_ipv4addr_cmp(conn->u.ipv4.laddr, INADDR_ANY))
    {
      conn->dev = NULL;
      return OK;
    }

  /* There are multiple network devices.  We need to select the device that
   * is going to route the TCP packet based on the provided IP address.
   */

  conn->dev = netdev_findby_ipv4addr(conn->u.ipv4.laddr,
                                     conn->u.ipv4.laddr);

  /* Return success if we found the device */

  return conn->dev != NULL ? OK : -ENETUNREACH;

#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return (g_netdevices != NULL) ? OK : -ENETUNREACH;
#endif
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Function: tcp_find_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The locally bound address, laddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int tcp_find_ipv6_device(FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* Return success without using device notification if the locally bound
   * address is IN6ADDR_ANY.  In this case, there may be multiple devices
   * that can provide data so the exceptional events from any particular
   * device are not important.
   */

  if (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_allzeroaddr))
    {
      conn->dev = NULL;
      return OK;
    }

  /* There are multiple network devices.  We need to select the device that
   * is going to route the TCP packet based on the provided IP address.
   */

  conn->dev = netdev_findby_ipv6addr(conn->u.ipv6.laddr,
                                     conn->u.ipv6.laddr);

  /* Return success if we found the device */

  return conn->dev != NULL ? OK : -ENETUNREACH;
#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return (g_netdevices != NULL) ? OK : -ENETUNREACH;
#endif

}
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET && CONFIG_NET_TCP */
