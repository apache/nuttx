/****************************************************************************
 * net/socket/sendmsg.c
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
#include <sys/socket.h>
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <string.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/uio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_sendmsg_check
 *
 * Description:
 *   Check the socket and the message of a send.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Message to send
 *
 * Returned Value:
 *   Zero (OK) when the send can start, or a negated errno value.
 *
 ****************************************************************************/

static int psock_sendmsg_check(FAR struct socket *psock,
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

  if (msg->msg_iov == NULL ||
      (psock->s_type != SOCK_DGRAM && msg->msg_iov->iov_base == NULL))
    {
      return -EINVAL;
    }

  return OK;
}

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
 *   - It accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Message to send
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   sendmsg() will return 0. Otherwise, on any failure, a negated errno
 *   value is returned (see comments with sendmsg() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

ssize_t psock_sendmsg(FAR struct socket *psock, FAR const struct msghdr *msg,
                       int flags)
{
  int ret;

  ret = psock_sendmsg_check(psock, msg);
  if (ret < 0)
    {
      return ret;
    }

  /* Let logic specific to this address family handle the sendmsg()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
              psock->s_sockif->si_sendmsg != NULL);

  return psock->s_sockif->si_sendmsg(psock, msg, flags);
}

#ifdef CONFIG_BUILD_KERNEL
/****************************************************************************
 * Name: psock_sendmsg_user
 *
 * Description:
 *   psock_sendmsg() for a message in user memory.  The network can send
 *   the data from another task, where the memory of the caller is not
 *   mapped.  So copy the data, the address and the control data to kernel
 *   memory before the send.  A stream sends up to SOCKET_COPY_SIZE bytes
 *   per part, while each part goes out whole; a datagram is sent whole.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Message to send, in user memory
 *   flags    Send flags
 *
 * Returned Value:
 *   The number of bytes sent, or a negated errno value when no byte was
 *   sent.
 *
 ****************************************************************************/

ssize_t psock_sendmsg_user(FAR struct socket *psock,
                           FAR const struct msghdr *msg, int flags)
{
  struct sockaddr_storage kname;
  struct msghdr kmsg;
  struct iovec kiov;
  struct uio uio;
  FAR uint8_t *kbuf;
  FAR void *kctrl = NULL;
  size_t bufsize;
  size_t total = 0;
  size_t len;
  ssize_t ret;

  ret = psock_sendmsg_check(psock, msg);
  if (ret < 0)
    {
      return ret;
    }

  ret = uio_init(&uio, msg->msg_iov, msg->msg_iovlen);
  if (ret < 0)
    {
      return ret;
    }

  if (msg->msg_name != NULL && msg->msg_namelen > sizeof(kname))
    {
      return -EINVAL;
    }

  if (msg->msg_control != NULL && msg->msg_controllen > SOCKET_COPY_SIZE)
    {
      return -ENOBUFS;
    }

  if (psock->s_type == SOCK_STREAM)
    {
      bufsize = MIN(uio.uio_resid, SOCKET_COPY_SIZE);
    }
  else if (uio.uio_resid > SOCKET_DGRAM_MAX)
    {
      return -EMSGSIZE;
    }
  else
    {
      bufsize = uio.uio_resid;
    }

  kbuf = kmm_malloc(bufsize);
  if (kbuf == NULL)
    {
      return -ENOMEM;
    }

  if (msg->msg_control != NULL && msg->msg_controllen > 0)
    {
      kctrl = kmm_malloc(msg->msg_controllen);
      if (kctrl == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      memcpy(kctrl, msg->msg_control, msg->msg_controllen);
    }

  if (msg->msg_name != NULL)
    {
      memcpy(&kname, msg->msg_name, msg->msg_namelen);
    }

  kiov.iov_base       = kbuf;
  kmsg.msg_name       = msg->msg_name != NULL ? &kname : NULL;
  kmsg.msg_namelen    = msg->msg_namelen;
  kmsg.msg_iov        = &kiov;
  kmsg.msg_iovlen     = 1;
  kmsg.msg_control    = kctrl;
  kmsg.msg_controllen = kctrl != NULL ? msg->msg_controllen : 0;
  kmsg.msg_flags      = msg->msg_flags;

  do
    {
      kiov.iov_len = MIN(uio.uio_resid - total, bufsize);
      uio_copyto(&uio, total, kbuf, kiov.iov_len);

      ret = psock_sendmsg(psock, &kmsg, flags);
      if (ret <= 0)
        {
          break;
        }

      len    = ret;
      total += len;

      /* The control data goes with the first part only */

      kmsg.msg_control    = NULL;
      kmsg.msg_controllen = 0;
    }
  while (len == kiov.iov_len && total < uio.uio_resid);

  /* Return the error only when no byte went out, as a send does */

  if (total > 0)
    {
      ret = total;
    }

out:
  kmm_free(kctrl);
  kmm_free(kbuf);
  return ret;
}
#endif

/****************************************************************************
 * Name: psock_sigpipe
 *
 * Description:
 *   Raise SIGPIPE in the calling thread when a send on a stream socket
 *   failed with -EPIPE, unless 'flags' has MSG_NOSIGNAL.  Kernel threads do
 *   not get the signal.
 *
 * Input Parameters:
 *   psock    The socket of the send
 *   ret      The result of the send
 *   flags    The send flags
 *
 ****************************************************************************/

void psock_sigpipe(FAR struct socket *psock, ssize_t ret, int flags)
{
#ifndef CONFIG_DISABLE_ALL_SIGNALS
  if (ret == -EPIPE && (flags & MSG_NOSIGNAL) == 0 &&
      (psock->s_type == SOCK_STREAM || psock->s_type == SOCK_SEQPACKET) &&
      (nxsched_self()->flags & TCB_FLAG_TTYPE_MASK) !=
      TCB_FLAG_TTYPE_KERNEL)
    {
      nxsig_tgkill(nxsched_getpid(), nxsched_gettid(), SIGPIPE);
    }
#endif
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

ssize_t sendmsg(int sockfd, FAR const struct msghdr *msg, int flags)
{
  FAR struct socket *psock;
  FAR struct file *filep;
  ssize_t ret;

  /* sendmsg() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &filep, &psock);

  /* Let psock_sendmsg_user() do all of the work */

  if (ret == OK)
    {
      ret = psock_sendmsg_user(psock, msg, flags);
      psock_sigpipe(psock, ret, flags);
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
