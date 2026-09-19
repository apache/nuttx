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

  /* Verify that non-NULL pointers were passed */

  if (msg == NULL || msg->msg_iov == NULL || msg->msg_iov->iov_base == NULL)
    {
      return -EINVAL;
    }

  if (msg->msg_name != NULL && msg->msg_namelen <= 0)
    {
      return -EINVAL;
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

  /* Save the original cmsg information */

  msg_control         = msg->msg_control;
  msg_controllen      = msg->msg_controllen;

  ret = psock->s_sockif->si_recvmsg(psock, msg, flags);

  /* Recover the pointer and calculate the cmsg's true data length */

  msg->msg_control    = msg_control;
  msg->msg_controllen = msg_controllen - msg->msg_controllen;

  return ret;
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
  FAR struct socket *psock;
  FAR struct file *filep;
  ssize_t ret;
#ifdef CONFIG_BUILD_KERNEL
  struct msghdr kmsg;
  struct msghdr umsg;
  FAR struct msghdr *recv_msg = msg;
  FAR struct iovec *user_iov = NULL;
  FAR struct iovec *kiov = NULL;
  FAR uint8_t *kdata = NULL;
  FAR void *kname = NULL;
  FAR void *kcontrol = NULL;
  size_t total_len;
  size_t i;
  FAR uint8_t *ptr;
  ssize_t remaining;
  size_t chunk;
#endif

  /* recvmsg() is a cancellation point */

  enter_cancellation_point();

#ifdef CONFIG_BUILD_KERNEL
  /* Receive into kernel buffers, then copy out to the user msghdr (see
   * recvfrom()).
   */

  memcpy(&umsg, msg, sizeof(struct msghdr));

  if (umsg.msg_iov == NULL)
    {
      ret = -EINVAL;
      goto errout_with_cleanup;
    }

  if (umsg.msg_iovlen > 0)
    {
      user_iov = kmm_malloc(umsg.msg_iovlen * sizeof(struct iovec));
      if (user_iov == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

      memcpy(user_iov, msg->msg_iov, umsg.msg_iovlen * sizeof(struct iovec));

      kiov = kmm_malloc(umsg.msg_iovlen * sizeof(struct iovec));
      if (kiov == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

      total_len = 0;
      for (i = 0; i < umsg.msg_iovlen; i++)
        {
          total_len += user_iov[i].iov_len;
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
              if (user_iov[i].iov_len > 0)
                {
                  kiov[i].iov_base = ptr;
                  kiov[i].iov_len = user_iov[i].iov_len;
                  ptr += user_iov[i].iov_len;
                }
              else
                {
                  kiov[i].iov_base = NULL;
                  kiov[i].iov_len = 0;
                }
            }
        }
      else
        {
          for (i = 0; i < umsg.msg_iovlen; i++)
            {
              kiov[i].iov_base = NULL;
              kiov[i].iov_len = 0;
            }
        }
    }

  memcpy(&kmsg, &umsg, sizeof(kmsg));
  kmsg.msg_iov = kiov;
  kmsg.msg_iovlen = umsg.msg_iovlen;

  if (umsg.msg_name != NULL && umsg.msg_namelen > 0)
    {
      kname = kmm_malloc(umsg.msg_namelen);
      if (kname == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_cleanup;
        }

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

      kmsg.msg_control = kcontrol;
    }
  else
    {
      kmsg.msg_control = NULL;
      kmsg.msg_controllen = 0;
    }

  recv_msg = &kmsg;
#endif

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &filep, &psock);

  /* Let psock_recvmsg() do all of the work */

  if (ret == OK)
    {
#ifdef CONFIG_BUILD_KERNEL
      ret = psock_recvmsg(psock, recv_msg, flags);
#else
      ret = psock_recvmsg(psock, msg, flags);
#endif
      file_put(filep);
    }

#ifdef CONFIG_BUILD_KERNEL
  if (ret >= 0)
    {
      remaining = ret;
      for (i = 0; i < umsg.msg_iovlen && remaining > 0; i++)
        {
          chunk = kiov[i].iov_len < (size_t)remaining ? kiov[i].iov_len :
                  (size_t)remaining;
          memcpy(user_iov[i].iov_base, kiov[i].iov_base, chunk);
          remaining -= (ssize_t)chunk;
        }

      msg->msg_namelen = kmsg.msg_namelen;
      if (umsg.msg_name != NULL && kmsg.msg_namelen > 0)
        {
          memcpy(umsg.msg_name, kname, kmsg.msg_namelen);
        }

      msg->msg_flags = kmsg.msg_flags;

      if (umsg.msg_control != NULL && umsg.msg_controllen > 0)
        {
          memcpy(umsg.msg_control, kcontrol, kmsg.msg_controllen);
        }

      msg->msg_controllen = kmsg.msg_controllen;
    }

errout_with_cleanup:
  kmm_free(kcontrol);
  kmm_free(kname);
  kmm_free(kdata);
  kmm_free(kiov);
  kmm_free(user_iov);
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
