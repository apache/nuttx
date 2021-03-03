/****************************************************************************
 * net/socket/recvmsg.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET_CMSG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_recvmsg
 *
 * Description:
 *   psock_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   recvfrom() except that:
 *
 *   - It is not a cancellation point,
 *   - It does not modify the errno variable, and
 *   - I accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock   - A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive msg
 *   len     - Length of buffer
 *   flags   - Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   psock_recvmsg() will return 0.  Otherwise, on any failure, a negated
 *   errno value is returned (see comments with recvmsg() for a list of
 *   appropriate errno values).
 *
 ****************************************************************************/

ssize_t psock_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags)
{
  /* Verify that non-NULL pointers were passed */

  if (msg == NULL)
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

  /* Let logic specific to this address family handle the recvfrom()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
              (psock->s_sockif->si_recvmsg != NULL ||
               psock->s_sockif->si_recvfrom != NULL));

  if (psock->s_sockif->si_recvmsg != NULL)
    {
      return psock->s_sockif->si_recvmsg(psock, msg, flags);
    }
  else
    {
      /* Socket doesn't implement si_recvmsg fallback to si_recvfrom */

      FAR void *buf             = msg->msg_iov->iov_base;
      FAR struct sockaddr *from = msg->msg_name;
      FAR socklen_t *fromlen    = (FAR socklen_t *)&msg->msg_namelen;
      size_t len                = msg->msg_iov->iov_len;

      return psock->s_sockif->si_recvfrom(psock, buf, len, flags, from,
                                          fromlen);
    }
}

/****************************************************************************
 * Name: nx_recvfrom
 *
 * Description:
 *   nx_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   recvfrom() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno variable.
 *
 * Input Parameters:
 *   sockfd  - Socket descriptor of socket
 *   msg      Buffer to receive msg
 *   len     - Length of buffer
 *   flags   - Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   nx_recvmsg() will return 0.  Otherwise, on any failure, a negated errno
 *   value is returned (see comments with recvmsg() for a list of appropriate
 *   errno values).
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
 *   The recvmsg() call is identical to recvfrom() with a NULL from
 *   parameter.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   msg      Buffer to receive msg
 *   len      Length of buffer
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
  FAR struct socket *psock;
  ssize_t ret;

  /* recvfrom() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Let psock_recvfrom() do all of the work */

  ret = psock_recvmsg(psock, msg, flags);
  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET_CMSG */
