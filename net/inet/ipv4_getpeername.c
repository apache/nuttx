/****************************************************************************
 * net/inet/ipv4_getpeername.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

  if ((psock->s_flags & _SF_CONNECTED) == 0)
    {
      return -ENOTCONN;
    }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef NET_TCP_HAVE_STACK
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *tcp_conn = (FAR struct tcp_conn_s *)psock->s_conn;

          outaddr->sin_port = tcp_conn->rport; /* Already in network byte order */
          ripaddr           = tcp_conn->u.ipv4.raddr;
        }
        break;
#endif

#ifdef NET_UDP_HAVE_STACK
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *udp_conn = (FAR struct udp_conn_s *)psock->s_conn;

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
  *addrlen = sizeof(struct sockaddr_in);

  /* Return success */

  return OK;
#else
  return -EOPNOTSUPP;
#endif
}

#endif /* CONFIG_NET_IPv4 */
