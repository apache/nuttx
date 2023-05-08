/****************************************************************************
 * net/inet/ipv4_getpeername.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "socket/socket.h"
#include "udp/udp.h"
#include "tcp/tcp.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_IPv4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_getpeername
 *
 * Description:
 *   The ipv4_getpeername() function retrieves the locally-bound name of the
 *   specified PF_NET socket.
 *
 * Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getpeername() for the list of returned error values.
 *
 ****************************************************************************/

int ipv4_getpeername(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen)
{
#if defined(NET_TCP_HAVE_STACK) || defined(NET_UDP_HAVE_STACK)
  FAR struct sockaddr_in *outaddr = (FAR struct sockaddr_in *)addr;
  FAR struct socket_conn_s *conn = psock->s_conn;
  in_addr_t ripaddr;

  /* Check if enough space has been provided for the full address */

  if (*addrlen < sizeof(struct sockaddr_in))
    {
      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided.  This support has not
       * been implemented.
       */

      return -ENOSYS;
    }

  /* Verify that the socket has been connected */

  if (_SS_ISCONNECTED(conn->s_flags) == 0)
    {
      return -ENOTCONN;
    }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef NET_TCP_HAVE_STACK
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *tcp_conn = psock->s_conn;

          outaddr->sin_port = tcp_conn->rport; /* Already in network byte order */
          ripaddr           = tcp_conn->u.ipv4.raddr;
        }
        break;
#endif

#ifdef NET_UDP_HAVE_STACK
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *udp_conn = psock->s_conn;

          outaddr->sin_port = udp_conn->rport; /* Already in network byte order */
          ripaddr           = udp_conn->u.ipv4.raddr;
        }
        break;
#endif

      default:
        return -EOPNOTSUPP;
    }

  /* Set the address family and the IP address */

  outaddr->sin_family      = psock->s_domain;
  outaddr->sin_addr.s_addr = ripaddr;
  memset(outaddr->sin_zero, 0, sizeof(outaddr->sin_zero));
  *addrlen = sizeof(struct sockaddr_in);

  /* Return success */

  return OK;
#else
  return -EOPNOTSUPP;
#endif
}

#endif /* CONFIG_NET_IPv4 */
