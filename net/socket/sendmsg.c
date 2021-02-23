/****************************************************************************
 * net/socket/sendmsg.c
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
 * Name: psock_sendmsg
 *
 * Description:
 *   psock_sendfrom() sends messages to a socket, and may be used to
 *   send data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sendfrom() except that:
 *
 *   - It is not a cancellation point,
 *   - It does not modify the errno variable, and
 *   - I accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock   - A pointer to a NuttX-specific, internal socket structure
 *   msg     - Buffer to of the msg
 *   len     - Length of buffer
 *   flags   - Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   send() will return 0.  Otherwise, on any failure, a negated errno value
 *   is returned (see comments with send() for a list of appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t psock_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
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

  /* Let logic specific to this address family handle the sendmsg()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
              (psock->s_sockif->si_sendmsg != NULL ||
               psock->s_sockif->si_sendto != NULL));

  if (psock->s_sockif->si_sendmsg != NULL)
    {
      return psock->s_sockif->si_sendmsg(psock, msg, flags);
    }
  else
    {
      /* Socket doesn't implement si_sendmsg fallback to si_sendto */

      FAR void *buf           = msg->msg_iov->iov_base;
      FAR struct sockaddr *to = msg->msg_name;
      socklen_t tolen         = msg->msg_namelen;
      size_t len              = msg->msg_iov->iov_len;

      return psock->s_sockif->si_sendto(psock, buf, len, flags, to, tolen);
    }
}

/****************************************************************************
 * Name: nx_sendfrom
 *
 * Description:
 *   nx_sendfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sendfrom() except that:
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
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   send() will return 0.  Otherwise, on any failure, a negated errno value
 *   is returned (see comments with send() for a list of appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_sendmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_sendmsg() do all of the work */

  return psock_sendmsg(psock, msg, flags);
}

/****************************************************************************
 * Function: sendmsg
 *
 * Description:
 *   The sendmsg() call is identical to sendfrom() with a NULL from
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

ssize_t sendmsg(int sockfd, FAR struct msghdr *msg, int flags)
{
  FAR struct socket *psock;
  ssize_t ret;

  /* sendfrom() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Let psock_sendfrom() do all of the work */

  ret = psock_sendmsg(psock, msg, flags);
  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET_CMSG */
