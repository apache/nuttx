/****************************************************************************
 * net/socket/recvmsg.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/param.h>
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <string.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/uio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_recvmsg_check
 *
 * Description:
 *   Check the socket and the message of a receive.
 *
 * Input Parameters:
 *   psock     A pointer to a NuttX-specific, internal socket structure
 *   msg       Buffer to receive data
 *
 * Returned Value:
 *   Zero (OK) when the receive can start, or a negated errno value.
 *
 ****************************************************************************/

static int psock_recvmsg_check(FAR struct socket *psock,
                               FAR const struct msghdr *msg)
{
  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      return -EBADF;
    }

  /* Verify that non-NULL pointers were passed */

  if (msg == NULL)
    {
      return -EINVAL;
    }

  /* POSIX: fail with EMSGSIZE if msg_iovlen <= 0 or > IOV_MAX */

  if (msg->msg_iovlen == 0 || msg->msg_iovlen > IOV_MAX)
    {
      return -EMSGSIZE;
    }

  if (msg->msg_iov == NULL || msg->msg_iov->iov_base == NULL)
    {
      return -EINVAL;
    }

  if (msg->msg_name != NULL && msg->msg_namelen <= 0)
    {
      return -EINVAL;
    }

  return OK;
}

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
 *   psock_recvmsg() will return 0.  Otherwise, on any failure, a negated
 *   errno value is returned (see comments with recvmsg() for a list of
 *   appropriate errno values).
 *
 ****************************************************************************/

ssize_t psock_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                       int flags)
{
  unsigned long msg_controllen;
  FAR void *msg_control;
  int ret;

  ret = psock_recvmsg_check(psock, msg);
  if (ret < 0)
    {
      return ret;
    }

  /* Let logic specific to this address family handle the recvmsg()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
              psock->s_sockif->si_recvmsg != NULL);

  /* Save the original cmsg information */

  msg_control         = msg->msg_control;
  msg_controllen      = msg->msg_controllen;

  ret = psock->s_sockif->si_recvmsg(psock, msg, flags);

  /* Recover the pointer and calculate the cmsg's true data length */

  msg->msg_control    = msg_control;
  msg->msg_controllen = msg_controllen - msg->msg_controllen;

  return ret;
}

#ifdef CONFIG_BUILD_KERNEL
/****************************************************************************
 * Name: psock_recvmsg_user
 *
 * Description:
 *   psock_recvmsg() for a message in user memory.  The network can
 *   complete the receive in another task, where the memory of the caller
 *   is not mapped.  So receive the data, the address and the control data
 *   into kernel memory, and copy them to the caller after the receive.  A
 *   stream receives up to SOCKET_COPY_SIZE bytes per part; a datagram is
 *   received whole.
 *
 * Input Parameters:
 *   psock     A pointer to a NuttX-specific, internal socket structure
 *   msg       Buffer to receive data, in user memory
 *   flags     Receive flags
 *
 * Returned Value:
 *   The number of bytes received, or a negated errno value on failure.
 *
 ****************************************************************************/

ssize_t psock_recvmsg_user(FAR struct socket *psock, FAR struct msghdr *msg,
                           int flags)
{
  struct sockaddr_storage kname;
  struct msghdr kmsg;
  struct iovec kiov;
  struct uio uio;
  FAR uint8_t *kbuf;
  FAR void *kctrl = NULL;
  socklen_t namesize = 0;
  size_t ctrlsize = 0;
  size_t bufsize;
  size_t total;
  size_t len;
  ssize_t ret;

  ret = psock_recvmsg_check(psock, msg);
  if (ret < 0)
    {
      return ret;
    }

  ret = uio_init(&uio, msg->msg_iov, msg->msg_iovlen);
  if (ret < 0)
    {
      return ret;
    }

  bufsize = MIN(uio.uio_resid, psock->s_type == SOCK_STREAM ?
                SOCKET_COPY_SIZE : SOCKET_DGRAM_MAX);
  kbuf = kmm_malloc(bufsize);
  if (kbuf == NULL)
    {
      return -ENOMEM;
    }

  if (msg->msg_control != NULL && msg->msg_controllen > 0)
    {
      ctrlsize = MIN(msg->msg_controllen, SOCKET_COPY_SIZE);
      kctrl = kmm_malloc(ctrlsize);
      if (kctrl == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }
    }

  /* A socket type can return no address: then the caller gets zeros */

  if (msg->msg_name != NULL)
    {
      memset(&kname, 0, sizeof(kname));
      namesize = MIN(msg->msg_namelen, sizeof(kname));
    }

  kiov.iov_base       = kbuf;
  kiov.iov_len        = bufsize;
  kmsg.msg_name       = msg->msg_name != NULL ? &kname : NULL;
  kmsg.msg_namelen    = msg->msg_name != NULL ? namesize : msg->msg_namelen;
  kmsg.msg_iov        = &kiov;
  kmsg.msg_iovlen     = 1;
  kmsg.msg_control    = kctrl;
  kmsg.msg_controllen = ctrlsize;
  kmsg.msg_flags      = msg->msg_flags;

  ret = psock_recvmsg(psock, &kmsg, flags);
  if (ret < 0)
    {
      goto out;
    }

  /* The address, the control data and the flags come with the first
   * part.  A cut datagram can return its whole length.
   */

  if (msg->msg_name != NULL)
    {
      memcpy(msg->msg_name, &kname, MIN(kmsg.msg_namelen, namesize));
    }

  if (kctrl != NULL)
    {
      memcpy(msg->msg_control, kctrl, kmsg.msg_controllen);
    }

  msg->msg_namelen    = kmsg.msg_namelen;
  msg->msg_controllen = kmsg.msg_controllen;
  msg->msg_flags      = kmsg.msg_flags;

  len = MIN((size_t)ret, bufsize);
  uio_copyfrom(&uio, 0, kbuf, len);
  if (psock->s_type != SOCK_STREAM)
    {
      goto out;
    }

  /* With MSG_WAITALL, a stream receives more parts until the buffers are
   * full, or until a part ends early.
   */

  kmsg.msg_name       = NULL;
  kmsg.msg_namelen    = 0;
  kmsg.msg_control    = NULL;
  kmsg.msg_controllen = 0;

  for (total = len; (flags & (MSG_WAITALL | MSG_PEEK)) == MSG_WAITALL &&
                    len == kiov.iov_len && total < uio.uio_resid;
       total += len)
    {
      kiov.iov_len = MIN(uio.uio_resid - total, bufsize);
      ret = psock_recvmsg(psock, &kmsg, flags);
      if (ret <= 0)
        {
          break;
        }

      len = ret;
      uio_copyfrom(&uio, total, kbuf, len);
    }

  ret = total;

out:
  kmm_free(kctrl);
  kmm_free(kbuf);
  return ret;
}
#endif

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
  FAR struct socket *psock;
  FAR struct file *filep;
  ssize_t ret;

  /* recvmsg() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &filep, &psock);

  /* Let psock_recvmsg_user() do all of the work */

  if (ret == OK)
    {
      ret = psock_recvmsg_user(psock, msg, flags);
      file_put(filep);
    }

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET */
