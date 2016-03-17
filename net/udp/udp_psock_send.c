/****************************************************************************
 * net/udp/udp_psock_send.c
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

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_udp_send
 *
 * Description:
 *   Implements send() for connected UDP sockets
 *
 ****************************************************************************/

ssize_t psock_udp_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len)
{
  FAR struct udp_conn_s *conn;
  union
  {
    struct sockaddr     addr;
#ifdef CONFIG_NET_IPv4
    struct sockaddr_in  addr4;
#endif
#ifdef CONFIG_NET_IPv6
    struct sockaddr_in6 addr6;
#endif
  } to;
  socklen_t tolen;

  DEBUGASSERT(psock != NULL && psock->s_crefs > 0);
  DEBUGASSERT(psock->s_type != SOCK_DGRAM);

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn);

  /* Was the UDP socket connected via connect()? */

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      /* No, then it is not legal to call send() with this socket. */

      return -ENOTCONN;
    }

  /* Yes, then let psock_sendto to the work */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET)
#endif
    {
      tolen               = sizeof(struct sockaddr_in);
      to.addr4.sin_family = AF_INET;
      to.addr4.sin_port   = conn->rport;
      net_ipv4addr_copy(to.addr4.sin_addr.s_addr, conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else
#endif
    {
      tolen               = sizeof(struct sockaddr_in6);
      to.addr6.sin6_family = AF_INET6;
      to.addr6.sin6_port   = conn->rport;
      net_ipv6addr_copy(to.addr6.sin6_addr.s6_addr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */

  return psock_udp_sendto(psock, buf, len, 0, &to.addr, tolen);
}
