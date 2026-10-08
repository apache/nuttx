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

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <sys/socket.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

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
  /* Verify that non-NULL pointers were passed */

  if (msg == NULL || msg->msg_iov == NULL ||
      (psock->s_type != SOCK_DGRAM && msg->msg_iov->iov_base == NULL))
    {
      return -EINVAL;
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
              psock->s_sockif->si_sendmsg != NULL);

  return psock->s_sockif->si_sendmsg(psock, msg, flags);
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
#ifdef CONFIG_BUILD_KERNEL
  struct msghdr kmsg;
  struct msghdr umsg;
  FAR struct iovec *uiov = NULL;
  FAR uint8_t *kdata = NULL;
  FAR void *kname = NULL;
  FAR void *kcontrol = NULL;
  FAR const struct msghdr *send_msg = msg;
  size_t total_len;
  size_t i;
  FAR uint8_t *ptr;
#endif

  /* sendmsg() is a cancellation point */

  enter_cancellation_point();

#ifdef CONFIG_BUILD_KERNEL
  /* Copy user msghdr, iovec array, and payload into kernel memory so the
   * network stack can safely memcpy from iov bases (see sendto()).
   */

  memcpy(&umsg, msg, sizeof(struct msghdr));

  if (umsg.msg_iov == NULL)
    {
      ret = -EINVAL;
      goto errout_with_cleanup;
    }

  if (umsg.msg_iovlen > 0)
    {
      uiov = kmm_malloc(umsg.msg_iovlen * sizeof(struct iovec));
      if (uiov == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

      memcpy(uiov, msg->msg_iov, umsg.msg_iovlen * sizeof(struct iovec));

      total_len = 0;
      for (i = 0; i < umsg.msg_iovlen; i++)
        {
          total_len += uiov[i].iov_len;
        }

      if (total_len > 0)
        {
          kdata = kmm_malloc(total_len);
          if (kdata == NULL)
            {
              ret = -ENOMEM;
              goto errout_with_cleanup;
            }

          ptr = kdata;
          for (i = 0; i < umsg.msg_iovlen; i++)
            {
              if (uiov[i].iov_len > 0)
                {
                  memcpy(ptr, uiov[i].iov_base, uiov[i].iov_len);
                  uiov[i].iov_base = ptr;
                  ptr += uiov[i].iov_len;
                }
              else
                {
                  uiov[i].iov_base = NULL;
                }
            }
        }
      else
        {
          for (i = 0; i < umsg.msg_iovlen; i++)
            {
              uiov[i].iov_base = NULL;
            }
        }
    }

  memcpy(&kmsg, &umsg, sizeof(kmsg));
  kmsg.msg_iov = uiov;
  kmsg.msg_iovlen = umsg.msg_iovlen;

  if (umsg.msg_name != NULL && umsg.msg_namelen > 0)
    {
      kname = kmm_malloc(umsg.msg_namelen);
      if (kname == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

      memcpy(kname, umsg.msg_name, umsg.msg_namelen);
      kmsg.msg_name = (FAR struct sockaddr *)kname;
    }
  else
    {
      kmsg.msg_name = NULL;
      kmsg.msg_namelen = 0;
    }

  if (umsg.msg_control != NULL && umsg.msg_controllen > 0)
    {
      kcontrol = kmm_malloc(umsg.msg_controllen);
      if (kcontrol == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

      memcpy(kcontrol, umsg.msg_control, umsg.msg_controllen);
      kmsg.msg_control = kcontrol;
    }
  else
    {
      kmsg.msg_control = NULL;
      kmsg.msg_controllen = 0;
    }

  send_msg = &kmsg;
#endif

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &filep, &psock);

  /* Let psock_sendmsg() do all of the work */

  if (ret == OK)
    {
#ifdef CONFIG_BUILD_KERNEL
      ret = psock_sendmsg(psock, send_msg, flags);
#else
      ret = psock_sendmsg(psock, msg, flags);
#endif
      file_put(filep);
    }

#ifdef CONFIG_BUILD_KERNEL
errout_with_cleanup:
  kmm_free(kcontrol);
  kmm_free(kname);
  kmm_free(kdata);
  kmm_free(uiov);
#endif

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET */
