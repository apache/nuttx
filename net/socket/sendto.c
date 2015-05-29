/****************************************************************************
 * net/socket/sendto.c
 *
 *   Copyright (C) 2007-2009, 2011-2015 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "udp/udp.h"
#include "local/local.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_sendto(FAR struct socket *psock, FAR const void *buf,
                     size_t len, int flags, FAR const struct sockaddr *to,
                     socklen_t tolen)
{
  socklen_t minlen;
#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
  ssize_t nsent;
#endif
  int err;

  /* If to is NULL or tolen is zero, then this function is same as send (for
   * connected socket types)
   */

  if (!to || !tolen)
    {
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM)
      return psock_send(psock, buf, len, flags);
#else
      ndbg("ERROR: No 'to' address\n");
      err = EINVAL;
      goto errout;
#endif
    }

  /* Verify that a valid address has been provided */

  switch (to->sa_family)
    {
#ifdef CONFIG_NET_IPv4
    case AF_INET:
      minlen = sizeof(struct sockaddr_in);
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case AF_INET6:
      minlen = sizeof(struct sockaddr_in6);
      break;
#endif

#ifdef CONFIG_NET_LOCAL_DGRAM
    case AF_LOCAL:
      minlen = sizeof(sa_family_t);
      break;
#endif

    default:
      ndbg("ERROR: Unrecognized address family: %d\n", to->sa_family);
      err = EAFNOSUPPORT;
      goto errout;
    }

  if (tolen < minlen)
    {
      ndbg("ERROR: Invalid address length: %d < %d\n", tolen, minlen);
      err = EBADF;
      goto errout;
    }

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ndbg("ERROR: Invalid socket\n");
      err = EBADF;
      goto errout;
    }

  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      ndbg("ERROR: Connected socket\n");
      err = EISCONN;
      goto errout;
    }

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL_DGRAM)
  /* Now handle the sendto() operation according to the socket domain,
   * currently either IP or Unix domains.
   */

#ifdef CONFIG_NET_LOCAL_DGRAM
#ifdef CONFIG_NET_UDP
  if (psock->s_domain == PF_LOCAL)
#endif
    {
      nsent = psock_local_sendto(psock, buf, len, flags, to, tolen);
    }
#endif /* CONFIG_NET_LOCAL_DGRAM */

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL_DGRAM
  else
#endif
    {
      nsent = psock_udp_sendto(psock, buf, len, flags, to, tolen);
    }
#endif /* CONFIG_NET_UDP */

  /* Check if the domain-specific sendto() logic failed */

  if (nsent < 0)
    {
      ndbg("ERROR: UDP or Unix domain sendto() failed: %ld\n", (long)nsent);
      err = -nsent;
      goto errout;
    }

  return nsent;
#else
  err = ENOSYS;
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL_DGRAM */

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t sendto(int sockfd, FAR const void *buf, size_t len, int flags,
               FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* And let psock_sendto do all of the work */

  return psock_sendto(psock, buf, len, flags, to, tolen);
}

#endif /* CONFIG_NET */
