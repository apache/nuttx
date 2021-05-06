/****************************************************************************
 * net/udp/udp_setsockopt.c
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/udp.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "utils/utils.h"
#include "netdev/netdev.h"
#include "udp/udp.h"

#ifdef CONFIG_NET_UDPPROTO_OPTIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_setsockopt
 *
 * Description:
 *   udp_setsockopt() sets the UDP-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/udp.h> for the a complete list of values of UDP protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int udp_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len)
{
#ifdef CONFIG_NET_UDP_BINDTODEVICE
  /* Keep alive options are the only UDP protocol socket option currently
   * supported.
   */

  FAR struct udp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && psock->s_conn != NULL);
  conn = (FAR struct udp_conn_s *)psock->s_conn;

  /* All of the UDP protocol options apply only UDP sockets.  The sockets
   * do not have to be connected.. that might occur later with the KeepAlive
   * already configured.
   */

  if (psock->s_type != SOCK_DGRAM)
    {
      nerr("ERROR:  Not a UDP socket\n");
      return -ENOTCONN;
    }

  /* Handle the UDP-protocol options */

  switch (option)
    {
#ifdef CONFIG_NET_UDP_BINDTODEVICE
      /* Handle the UDP_BINDTODEVICE socket-level option.
       *
       * NOTE: UDP_BINDTODEVICE is declared in linux as SO_BINDTODEVICE,
       * but this option only makes sense for UDP sockets trying to broadcast
       * while their local address is not set, eg, with DHCP requests.
       * The problem is that we are not able to determine the interface to be
       * used for sending packets when multiple interfaces do not have a
       * local address yet. This option can be used to "force" the interface
       * used to send the UDP traffic in this connection. Note that it does
       * NOT only apply to broadcast packets.
       */

      case UDP_BINDTODEVICE:  /* Bind socket to a specific network device */
        {
          FAR struct net_driver_s *dev;

          /* Check if we are are unbinding the socket */

          if (value == NULL || value_len == 0 ||
             (value_len > 0 && ((FAR char *)value)[0] == 0))
            {
              /* Just report success if the socket is not bound to an
               * interface.
               */

              if (conn->boundto != 0)
                {
                  /* Get the interface that we are bound do.  NULL would
                   * indicate that the interface no longer exists for some
                   * reason.
                   */

                  dev = netdev_findbyindex(conn->boundto);
                  if (dev != NULL)
                    {
                      /* Clear the interface flag to unbind the device from
                       * the socket.
                       */

                      IFF_CLR_BOUND(dev->d_flags);
                    }

                  conn->boundto = 0;  /* This interface is no longer bound */
                }

              ret = OK;
            }

          /* No, we are binding a socket to the interface. */

          else
            {
              /* Find the interface device with this name */

              dev = netdev_findbyname(value);
              if (dev == NULL)
                {
                  ret = -ENODEV;
                }

              /* An interface may be bound only to one socket. */

              else if (IFF_IS_BOUND(dev->d_flags))
                {
                  ret = -EBUSY;
                }
              else
                {
                  /* Bind the interface to a socket */

                  IFF_SET_BOUND(dev->d_flags);

                  /* Bind the socket to the interface */

                  DEBUGASSERT(dev->d_ifindex > 0 &&
                              dev->d_ifindex <= MAX_IFINDEX);
                  conn->boundto = dev->d_ifindex;
                  ret = OK;
                }
            }
        }
        break;
#endif

      default:
        nerr("ERROR: Unrecognized UDP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
#else
  return -ENOPROTOOPT;
#endif /* CONFIG_NET_UDP_BINDTODEVICE */
}

#endif /* CONFIG_NET_UDPPROTO_OPTIONS */
