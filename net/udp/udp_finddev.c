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
#include "inet/inet.h"
#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_find_laddr_device
 *
 * Description:
 *   Select the network driver to use with the UDP transaction using the
 *   locally bound IP address.
 *
 *   This is currently used in the UDP network poll setup to determine
 *   which device is being polled.
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
  /* There are multiple network devices.  We need to select the device that
   * is going to route the UDP packet based on the provided IP address.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          /* Make sure that the socket is bound to some non-zero, local
           * address.  Zero is used as an indication that the laddr is
           * uninitialized and that the socket is, hence, not bound.
           */

          if (conn->u.ipv4.laddr == 0)
            {
              return NULL;
            }
          else
            {
              return netdev_findby_ipv4addr(conn->u.ipv4.laddr,
                                            conn->u.ipv4.laddr);
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          /* Make sure that the socket is bound to some non-zero, local
           * address.  The IPv6 unspecified address is used as an indication
           * that the laddr is uninitialized and that the socket is, hence,
           * not bound.
           */

          if (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
            {
              return NULL;
            }
          else
            {
              return netdev_findby_ipv6addr(conn->u.ipv6.laddr,
                                            conn->u.ipv6.laddr);
            }
        }
#endif
}

/****************************************************************************
 * Name: udp_find_raddr_device
 *
 * Description:
 *   Select the network driver to use with the UDP transaction using the
 *   remote IP address.
 *
 *   This function is called for UDP sendto() in order to determine which
 *   network device that the UDP pack should be sent on.
 *
 * Input Parameters:
 *   conn - UDP connection structure.
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

FAR struct net_driver_s *udp_find_raddr_device(FAR struct udp_conn_s *conn)
{
  /* We need to select the device that is going to route the UDP packet
   * based on the provided IP address.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          /* Check if the remote, destination address is the broadcast
           * or multicast address.  If this is the case, select the device
           * using the locally bound address (assuming that there is one).
           */

          if (conn->u.ipv4.raddr == INADDR_BROADCAST ||
              IN_MULTICAST(conn->u.ipv4.raddr))
            {
              /* Make sure that the socket is bound to some non-zero, local
               * address.  Zero is used as an indication that the laddr is
               * uninitialized and that the socket is, hence, not bound.
               */

              if (conn->u.ipv4.laddr == 0)
                {
                  return NULL;
                }
              else
                {
                  return netdev_findby_ipv4addr(conn->u.ipv4.laddr,
                                                conn->u.ipv4.laddr);
                }
            }

          /* Normal lookup using the remote address */

          else
            {
              return netdev_findby_ipv4addr(conn->u.ipv4.laddr,
                                            conn->u.ipv4.raddr);
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          /* Check if the remote, destination address is a multicast
           * address.  If this is the case, select the device
           * using the locally bound address (assuming that there is one).
           */

          if (net_is_addr_mcast(conn->u.ipv6.raddr))
            {
              /* Make sure that the socket is bound to some non-zero, local
               * address.  The IPv6 unspecified address is used as an
               * indication that the laddr is uninitialized and that the
               * socket is, hence, not bound.
               */

              if (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
                {
                  return NULL;
                }
              else
                {
                  return netdev_findby_ipv6addr(conn->u.ipv6.laddr,
                                                conn->u.ipv6.laddr);
                }
            }

          /* Normal lookup using the remote address */

          else
            {
              return netdev_findby_ipv6addr(conn->u.ipv6.laddr,
                                            conn->u.ipv6.raddr);
            }
        }
#endif
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
