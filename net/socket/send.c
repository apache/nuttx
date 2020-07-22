/****************************************************************************
 * net/socket/send.c
 *
 *   Copyright (C) 2007-2014, 2016-2017 Gregory Nutt. All rights reserved.
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
#include <sys/socket.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_send
 *
 * Description:
 *   The psock_send() call may be used only when the socket is in a
 *   connected state (so that the intended recipient is known).  This is an
 *   internal OS interface.  It is functionally equivalent to send() except
 *   that:
 *
 *   - It is not a cancellation point,
 *   - It does not modify the errno variable, and
 *   - I accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 *   See comments with send() for more a more complete description of the
 *   functionality.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   len   - Length of data to send
 *   flags - Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any failure, a
 *   negated errno value is returned (See comments with send() for a list
 *   of the appropriate errno value).
 *
 ****************************************************************************/

ssize_t psock_send(FAR struct socket *psock, FAR const void *buf, size_t len,
                   int flags)
{
  ssize_t ret;

  /* Verify that non-NULL pointers were passed */

  if (buf == NULL)
    {
      return -EINVAL;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Let the address family's send() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL && psock->s_sockif->si_send != NULL);

  ret = psock->s_sockif->si_send(psock, buf, len, flags);
  if (ret < 0)
    {
      nerr("ERROR: socket si_send() (or usrsock_sendto()) failed: %d\n",
           ret);
    }

  return ret;
}

/****************************************************************************
 * Name: nx_send
 *
 * Description:
 *   The nx_send() call may be used only when the socket is in a
 *   connected state (so that the intended recipient is known).  This is an
 *   internal OS interface.  It is functionally equivalent to send() except
 *   that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno variable.
 *
 *   See comments with send() for more a more complete description of the
 *   functionality.
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket
 *   buf    - Data to send
 *   len    - Length of data to send
 *   flags  - Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any failure, a
 *   negated errno value is returned (See comments with send() for a list
 *   of the appropriate errno value).
 *
 ****************************************************************************/

ssize_t nx_send(int sockfd, FAR const void *buf, size_t len, int flags)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* And let psock_send do all of the work */

  return psock_send(psock, buf, len, flags);
}

/****************************************************************************
 * Name: send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket
 *   buf    - Data to send
 *   len    - Length of data to send
 *   flags  - Send flags
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

ssize_t send(int sockfd, FAR const void *buf, size_t len, int flags)
{
  FAR struct socket *psock;
  ssize_t ret;

  /* send() is a cancellation point */

  enter_cancellation_point();

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Let psock_send() do all of the work */

  ret = psock_send(psock, buf, len, flags);
  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
