/****************************************************************************
 * net/socket/recvfrom.c
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

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_recvfrom
 *
 * Description:
 *   psock_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *   This is an internal OS interface.  It is functionally equivalent to
 *   recvfrom() except that:
 *
 *   - It is not a cancellation point,
 *   - It does not modify the errno variable, and
 *   - It accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock     A pointer to a NuttX-specific, internal socket structure
 *   buf       Buffer to receive data
 *   len       Length of buffer
 *   flags     Receive flags
 *   from      Address of source (may be NULL)
 *   fromlen   The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   psock_recvfrom() will return 0.  Otherwise, on any failure, a negated
 *   errno value is returned (see comments with recvfrom() for a list of
 *   appropriate errno values).
 *
 ****************************************************************************/

ssize_t psock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                       int flags, FAR struct sockaddr *from,
                       FAR socklen_t *fromlen)
{
  struct msghdr msg;
  struct iovec iov;
  ssize_t ret;

  iov.iov_base = buf;
  iov.iov_len = len;
  msg.msg_name = from;
  msg.msg_namelen = fromlen ? *fromlen : 0;
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = NULL;
  msg.msg_controllen = 0;
  msg.msg_flags = 0;

  /* And let psock_recvmsg do all of the work */

  ret = psock_recvmsg(psock, &msg, flags);
  if (ret >= 0 && fromlen != NULL)
    *fromlen = msg.msg_namelen;

  return ret;
}

/****************************************************************************
 * Name: recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   sockfd    Socket descriptor of socket
 *   buf       Buffer to receive data
 *   len       Length of buffer
 *   flags     Receive flags
 *   from      Address of source (may be NULL)
 *   fromlen   The length of the address structure
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

ssize_t recvfrom(int sockfd, FAR void *buf, size_t len, int flags,
                 FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct socket *psock;
  ssize_t ret;

  /* recvfrom() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &psock);

  /* Then let psock_recvfrom() do all of the work */

  if (ret == OK)
    {
      ret = psock_recvfrom(psock, buf, len, flags, from, fromlen);
    }

  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET */
