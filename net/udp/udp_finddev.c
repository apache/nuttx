/****************************************************************************
 * net/udp/udp_finddev.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "udp/udp.h"

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
 * Function: udp_find_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 UDP transaction.
 *
 * Input Parameters:
 *   conn - UDP connection structure (not currently used).
 *   ipv4addr - The IPv4 address to use in the device selection.
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct net_driver_s *udp_find_ipv4_device(FAR struct udp_conn_s *conn,
                                              in_addr_t ipv4addr)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* Return NULL if the address is INADDR_ANY.  In this case, there may
   * be multiple devices that can provide data so the exceptional events
   * from any particular device are not important.
   *
   * Of course, it would be a problem if this is the remote address of
   * sendto().
   */

  if (net_ipv4addr_cmp(ipv4addr, INADDR_ANY))
    {
      return NULL;
    }

  /* There are multiple network devices.  We need to select the device that
   * is going to route the UDP packet based on the provided IP address.
   */

  return netdev_findby_ipv4addr(conn->u.ipv4.laddr, ipv4addr);

#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return g_netdevices;
#endif
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Function: udp_find_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 UDP transaction.
 *
 * Input Parameters:
 *   conn - UDP connection structure (not currently used).
 *   ipv6addr - The IPv6 address to use in the device selection.
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct net_driver_s *udp_find_ipv6_device(FAR struct udp_conn_s *conn,
                                              net_ipv6addr_t ipv6addr)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* Return NULL if the address is IN6ADDR_ANY.  In this case, there may
   * be multiple devices that can provide data so the exceptional events
   * from any particular device are not important.
   *
   * Of course, it would be a problem if this is the remote address of
   * sendto().
   */

  if (net_ipv6addr_cmp(ipv6addr, g_ipv6_allzeroaddr))
    {
      return NULL;
    }

  /* There are multiple network devices.  We need to select the device that
   * is going to route the UDP packet based on the provided IP address.
   */

  return netdev_findby_ipv6addr(conn->u.ipv6.laddr, ipv6addr);

#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return g_netdevices;
#endif
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Function: udp_find_laddr_device
 *
 * Description:
 *   Select the network driver to use with the UDP transaction using the
 *   locally bound IP address.
 *
 * Input Parameters:
 *   conn - UDP connection structure (not currently used).
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

FAR struct net_driver_s *udp_find_laddr_device(FAR struct udp_conn_s *conn)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* There are multiple network devices.  We need to select the device that
   * is going to route the UDP packet based on the provided IP address.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          return udp_find_ipv4_device(conn, conn->u.ipv4.laddr);
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          return udp_find_ipv6_device(conn, conn->u.ipv6.laddr);
        }
#endif

#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return g_netdevices;
#endif
}

/****************************************************************************
 * Function: udp_find_raddr_device
 *
 * Description:
 *   Select the network driver to use with the UDP transaction using the
 *   remote IP address.
 *
 * Input Parameters:
 *   conn - UDP connection structure (not currently used).
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

FAR struct net_driver_s *udp_find_raddr_device(FAR struct udp_conn_s *conn)
{
#ifdef CONFIG_NETDEV_MULTINIC
  /* There are multiple network devices.  We need to select the device that
   * is going to route the UDP packet based on the provided IP address.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          return udp_find_ipv4_device(conn, conn->u.ipv4.raddr);
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          return udp_find_ipv6_device(conn, conn->u.ipv6.raddr);
        }
#endif

#else
  /* There is only a single network device... the one at the head of the
   * g_netdevices list.
   */

  return g_netdevices;
#endif
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
