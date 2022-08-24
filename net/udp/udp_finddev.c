/****************************************************************************
 * net/udp/udp_finddev.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "udp/udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: upd_bound_device
 *
 * Description:
 *   If the UDP socket is bound to a device, return the reference to the
 *   bound device.
 *
 * Input Parameters:
 *   conn - UDP connection structure (not currently used).
 *
 * Returned Value:
 *   A reference to the bound device.  If the retained interface index no
 *   longer refers to a valid device, this function will unbind the device
 *   and return an arbitrary network device at the head of the list of
 *   registered devices.  This supports legacy IPv4 DHCPD behavior when
 *   there is only a single registered network device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_BINDTODEVICE
static FAR struct net_driver_s *upd_bound_device(FAR struct udp_conn_s *conn)
{
  FAR struct net_driver_s *dev = NULL;

  /* Is the UDP socket bound to a device? */

  if (conn->sconn.s_boundto != 0)
    {
      /* Yes..This socket has been bound to an interface.  Convert the
       * interface index into a device structure reference.
       */

      dev = netdev_findbyindex(conn->sconn.s_boundto);
      if (dev == NULL)
        {
          /* No device?  It must have been unregistered.  Un-bind the UDP
           * socket.
           */

          conn->sconn.s_boundto = 0;
        }
    }

  /* If no device was bound or the bound device is no longer valid,
   * then let's try the default network device.
   */

  return dev == NULL ? netdev_default() : dev;
}
#else
#  define upd_bound_device(c) netdev_default();
#endif

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
 *   A pointer to the network driver to use.  NULL is returned if driver is
 *   not bound to any local device.
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
              return netdev_findby_ripv4addr(conn->u.ipv4.laddr,
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
              return netdev_findby_ripv6addr(conn->u.ipv6.laddr,
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

FAR struct net_driver_s *
udp_find_raddr_device(FAR struct udp_conn_s *conn,
                      FAR struct sockaddr_storage *remote)
{
  /* We need to select the device that is going to route the UDP packet
   * based on the provided IP address.
   */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (conn->domain == PF_INET)
#endif
        {
          in_addr_t raddr;

          if (remote)
            {
              FAR const struct sockaddr_in *inaddr =
                (FAR const struct sockaddr_in *)remote;
              net_ipv4addr_copy(raddr, inaddr->sin_addr.s_addr);
            }
          else
            {
              net_ipv4addr_copy(raddr, conn->u.ipv4.raddr);
            }

          /* Check if the remote, destination address is the broadcast
           * or multicast address.  If this is the case, select the device
           * using the locally bound address (assuming that there is one).
           */

          if (raddr == INADDR_BROADCAST || IN_MULTICAST(NTOHL(raddr)))
            {
              /* Make sure that the socket is bound to some non-zero, local
               * address.  Zero is used as an indication that the laddr is
               * uninitialized and that the socket is, hence, not bound.
               */

              if (conn->u.ipv4.laddr == 0) /* INADDR_ANY */
                {
                  /* Return the device bound to this UDP socket, if any */

                  return upd_bound_device(conn);
                }
              else
                {
                  return netdev_findby_ripv4addr(conn->u.ipv4.laddr,
                                                 conn->u.ipv4.laddr);
                }
            }

          /* There is no unique device associated with the unspecified
           * address.
           */

          else if (raddr != INADDR_ANY)
            {
              /* Normal lookup using the verified remote address */

              return netdev_findby_ripv4addr(conn->u.ipv4.laddr,
                                             raddr);
            }
          else
            {
              /* Not a suitable IPv4 unicast address for device lookup.
               * Return the device bound to this UDP socket, if any.
               */

              return upd_bound_device(conn);
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          net_ipv6addr_t raddr;

          if (remote)
            {
              FAR const struct sockaddr_in6 *inaddr =
                (FAR const struct sockaddr_in6 *)remote;
              net_ipv6addr_copy(raddr, inaddr->sin6_addr.s6_addr16);
            }
          else
            {
              net_ipv6addr_copy(raddr, conn->u.ipv6.raddr);
            }

          /* Check if the remote, destination address is a multicast
           * address.  If this is the case, select the device
           * using the locally bound address (assuming that there is one).
           */

          if (net_is_addr_mcast(raddr))
            {
              /* Make sure that the socket is bound to some non-zero, local
               * address.  The IPv6 unspecified address is used as an
               * indication that the laddr is uninitialized and that the
               * socket is, hence, not bound.
               */

              if (net_ipv6addr_cmp(conn->u.ipv6.laddr, g_ipv6_unspecaddr))
                {
                  /* Return the device bound to this UDP socket, if any */

                  return upd_bound_device(conn);
                }
              else
                {
                  return netdev_findby_ripv6addr(conn->u.ipv6.laddr,
                                                 conn->u.ipv6.laddr);
                }
            }

          /* There is no unique device associated with the unspecified
           * address.
           */

          else if (!net_ipv6addr_cmp(raddr, g_ipv6_unspecaddr))
            {
              /* Normal lookup using the verified remote address */

              return netdev_findby_ripv6addr(conn->u.ipv6.laddr,
                                             raddr);
            }
          else
            {
              /* Not a suitable IPv6 unicast address for device lookup.
               * Return the device bound to this UDP socket, if any.
               */

              return upd_bound_device(conn);
            }
        }
#endif
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
