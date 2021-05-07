/****************************************************************************
 * net/tcp/tcp_finddev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include "inet/inet.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_find_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP connection.
 *
 * Input Parameters:
 *   conn - TCP connection structure.
 *   addr - The IPv4 address to use
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static int tcp_find_ipv4_device(FAR struct tcp_conn_s *conn,
                                in_addr_t addr, bool local)
{
  /* Do nothing if a device is already bound to the connection */

  if (conn->dev != NULL)
    {
      return OK;
    }

  /* Return success without using device notification if the locally bound
   * address is INADDR_ANY.  In this case, there may be multiple devices
   * that can provide data so the exceptional events from any particular
   * device are not important.
   */

  if (net_ipv4addr_cmp(addr, INADDR_ANY))
    {
      return local ? OK : -EINVAL;
    }

  /* We need to select the device that is going to route the TCP packet
   * based on the provided IP address.
   */

  conn->dev = netdev_findby_ripv4addr(addr, addr);

  /* Return success if we found the device */

  return conn->dev != NULL ? OK : -ENETUNREACH;
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: tcp_find_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction.
 *
 * Input Parameters:
 *   conn - TCP connection structure.
 *   addr - The IPv6 address to use
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static int tcp_find_ipv6_device(FAR struct tcp_conn_s *conn,
                                const net_ipv6addr_t addr, bool local)
{
  /* Do nothing if a device is already bound to the connection */

  if (conn->dev != NULL)
    {
      return OK;
    }

  /* Return success without using device notification if the locally bound
   * address is the IPv6 unspecified address.  In this case, there may be
   * multiple devices* that can provide data so the exceptional events from
   * any particular device are not important.
   */

  if (net_ipv6addr_cmp(addr, g_ipv6_unspecaddr))
    {
      return local ? OK : -EINVAL;
    }

  /* We need to select the device that is going to route the TCP packet
   * based on the provided IP address.
   */

  conn->dev = netdev_findby_ripv6addr(addr, addr);

  /* Return success if we found the device */

  return conn->dev != NULL ? OK : -ENETUNREACH;
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_local_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction based
 *   on the locally bound IPv4 address.
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
int tcp_local_ipv4_device(FAR struct tcp_conn_s *conn)
{
  return tcp_find_ipv4_device(conn, conn->u.ipv4.laddr, true);
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: tcp_remote_ipv4_device
 *
 * Description:
 *   Select the network driver to use with the IPv4 TCP transaction based
 *   on the remotely connected IPv4 address.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The remotely connected address, raddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -ENETUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int tcp_remote_ipv4_device(FAR struct tcp_conn_s *conn)
{
  return tcp_find_ipv4_device(conn, conn->u.ipv4.raddr, false);
}
#endif

/****************************************************************************
 * Name: tcp_local_ipv6_device
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
int tcp_local_ipv6_device(FAR struct tcp_conn_s *conn)
{
  return tcp_find_ipv6_device(conn, conn->u.ipv6.laddr, true);
}
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: tcp_remote_ipv6_device
 *
 * Description:
 *   Select the network driver to use with the IPv6 TCP transaction based
 *   on the remotely connected IPv6 address.
 *
 * Input Parameters:
 *   conn - TCP connection structure.  The remotely connected address, raddr,
 *     should be set to a non-zero value in this structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.  -EHOSTUNREACH is the only expected error value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int tcp_remote_ipv6_device(FAR struct tcp_conn_s *conn)
{
  return tcp_find_ipv6_device(conn, conn->u.ipv6.raddr, false);
}
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET && CONFIG_NET_TCP */
