/****************************************************************************
 * net/inet/ipv6_getsockname.c
 *
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"
#include "udp/udp.h"
#include "tcp/tcp.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_IPv6

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_getsockname
 *
 * Description:
 *   The ipv6_getsockname() function retrieves the locally-bound name of the
 *   specified PF_NET6 socket.
 *
 * Input Parameters:
 *   psock    Point to the socket structure instance [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of returned error values.
 *
 ****************************************************************************/

int ipv6_getsockname(FAR struct socket *psock, FAR struct sockaddr *addr,
                     FAR socklen_t *addrlen)
{
  FAR struct sockaddr_in6 *outaddr = (FAR struct sockaddr_in6 *)addr;
  FAR struct net_driver_s *dev;
  net_ipv6addr_t *lipaddr;
  net_ipv6addr_t *ripaddr;

  /* Check if enough space has been provided for the full address */

  if (*addrlen < sizeof(struct sockaddr_in6))
    {
      /* This function is supposed to return the partial address if
       * a smaller buffer has been provided.  This support has not
       * been implemented.
       */

     return -ENOSYS;
   }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef NET_TCP_HAVE_STACK
      case SOCK_STREAM:
        {
          FAR struct tcp_conn_s *tcp_conn = (FAR struct tcp_conn_s *)psock->s_conn;

          outaddr->sin6_port = tcp_conn->lport; /* Already in network byte order */
          lipaddr            = &tcp_conn->u.ipv6.laddr;
          ripaddr            = &tcp_conn->u.ipv6.raddr;
        }
        break;
#endif

#ifdef NET_UDP_HAVE_STACK
      case SOCK_DGRAM:
        {
          FAR struct udp_conn_s *udp_conn = (FAR struct udp_conn_s *)psock->s_conn;

          outaddr->sin6_port = udp_conn->lport; /* Already in network byte order */
          lipaddr            = &udp_conn->u.ipv6.laddr;
          ripaddr            = &udp_conn->u.ipv6.raddr;
        }
        break;
#endif

      default:
        return -EOPNOTSUPP;
    }

  /* Check if bound to the IPv6 unspecified address */

  if (net_ipv6addr_cmp(lipaddr, g_ipv6_unspecaddr))
    {
      outaddr->sin6_family = AF_INET6;
      memcpy(outaddr->sin6_addr.in6_u.u6_addr8, g_ipv6_unspecaddr, 16);
      *addrlen = sizeof(struct sockaddr_in6);

      return OK;
    }

  net_lock();

  /* Find the device matching the IPv6 address in the connection structure.
   * NOTE:  listening sockets have no ripaddr.  Work around is to use the
   * lipaddr when ripaddr is not available.
   */

  if (net_ipv6addr_cmp(ripaddr, g_ipv6_unspecaddr))
    {
      ripaddr = lipaddr;
    }

  dev = netdev_findby_ipv6addr(*lipaddr, *ripaddr);
  if (!dev)
    {
      net_unlock();
      return -EINVAL;
    }

  /* Set the address family and the IP address */

  outaddr->sin6_family = AF_INET6;
  memcpy(outaddr->sin6_addr.in6_u.u6_addr8, dev->d_ipv6addr, 16);
  *addrlen = sizeof(struct sockaddr_in6);

  net_unlock();

  /* Return success */

  return OK;
}

#endif /* CONFIG_NET_IPv6 */
