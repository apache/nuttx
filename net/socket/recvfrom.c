/****************************************************************************
 * net/socket/recvfrom.c
 *
 *   Copyright (C) 2007-2009, 2011-2017 Gregory Nutt. All rights reserved.
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
 *   - I accepts the internal socket structure as an input rather than an
 *     task-specific socket descriptor.
 *
 * Input Parameters:
 *   psock   - A pointer to a NuttX-specific, internal socket structure
 *   buf     - Buffer to receive data
 *   len     - Length of buffer
 *   flags   - Receive flags
 *   from    - Address of source (may be NULL)
 *   fromlen - The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on any failure, a negated errno value
 *   is returned (see comments with send() for a list of appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t psock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                       int flags, FAR struct sockaddr *from,
                       FAR socklen_t *fromlen)
{
  ssize_t ret;

  /* Verify that non-NULL pointers were passed */

#ifdef CONFIG_DEBUG_FEATURES
  if (!buf)
    {
      return -EINVAL;
    }
#endif

  if (from != NULL && fromlen != NULL && *fromlen <= 0)
    {
      return -EINVAL;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Set the socket state to receiving */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_RECV);

  /* Let logic specific to this address family handle the recvfrom()
   * operation.
   */

  DEBUGASSERT(psock->s_sockif != NULL &&
             psock->s_sockif->si_recvfrom != NULL);

  ret = psock->s_sockif->si_recvfrom(psock, buf, len, flags, from, fromlen);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);
  return ret;
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
 *   buf     - Buffer to receive data
 *   len     - Length of buffer
 *   flags   - Receive flags
 *   from    - Address of source (may be NULL)
 *   fromlen - The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on any failure, a negated errno value
 *   is returned (see comments with send() for a list of appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_recvfrom(int sockfd, FAR void *buf, size_t len, int flags,
                    FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_recvfrom() do all of the work */

  return psock_recvfrom(psock, buf, len, flags, from, fromlen);
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
 *   sockfd  - Socket descriptor of socket
 *   buf     - Buffer to receive data
 *   len     - Length of buffer
 *   flags   - Receive flags
 *   from    - Address of source (may be NULL)
 *   fromlen - The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
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
  ssize_t ret;

  /* recvfrom() is a cancellation point */

  (void)enter_cancellation_point();

  /* Let nx_recvfrom and psock_recvfrom() do all of the work */

  ret = nx_recvfrom(sockfd, buf, len, flags, from, fromlen);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_NET */
