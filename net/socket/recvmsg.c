/****************************************************************************
 * net/socket/recvmsg.c
 *
 *   Copyright (C) 2007-2009, 2011-2017, 2019 Gregory Nutt. All rights
 *     reserved.
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

#include <assert.h>
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_recvmsg
 *
 * Description:
 *   psock_recvmsg() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   recvmsg() except that:
 *
 *   - It is not a cancellation point,
 *   - It does not modify the errno variable, and
 *   - It accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock     A pointer to a NuttX-specific, internal socket structure
 *   msg       Buffer to receive data
 *   flags     Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on any failure, a negated errno
 *   value is returned (see comments with recvmsg() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

ssize_t psock_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags)
{
  /* Verify that non-NULL pointers were passed */

  if (msg == NULL || msg->msg_iov == NULL || msg->msg_iov->iov_base == NULL)
    {
      return -EINVAL;
    }

  if (msg->msg_name != NULL && msg->msg_namelen <= 0)
    {
      return -EINVAL;
    }

  if (msg->msg_iovlen != 1)
    {
      return -ENOTSUP;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Let logic specific to this address family handle the recvmsg()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
              psock->s_sockif->si_recvmsg != NULL);

  return psock->s_sockif->si_recvmsg(psock, msg, flags);
}

/****************************************************************************
 * Name: nx_recvmsg
 *
 * Description:
 *   nx_recvmsg() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   recvmsg() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno variable.
 *
 * Input Parameters:
 *   sockfd  - Socket descriptor of socket
 *   msg      Buffer to receive the message
 *   flags   - Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on any failure, a negated errno value
 *   is returned (see comments with send() for a list of appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_recvmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_recvmsg() do all of the work */

  return psock_recvmsg(psock, msg, flags);
}

/****************************************************************************
 * Function: recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   msg      Buffer to receive the message
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would
 *     block, or a receive timeout had been set and the timeout expired
 *     before data was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically
 *     because it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address
 *     space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data
 *     were available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 ****************************************************************************/

ssize_t recvmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  ssize_t ret;

  /* recvmsg() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_recvmsg() do all of the work */

  ret = nx_recvmsg(sockfd, msg, flags);
  if (ret < 0)
    {
      _SO_SETERRNO(sockfd_socket(sockfd), -ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET */
