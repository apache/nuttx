/****************************************************************************
 * net/socket/accept.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_accept
 *
 * Description:
 *   The psock_accept function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an psock_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, psock_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, psock_accept returns
 *   EAGAIN.
 *
 * Input Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size
 *            of 'addr'
 *   newsock  Location to return the accepted socket information.
 *   flags    The flags used for initialization
 *
 * Returned Value:
 *  Returns 0 (OK) on success.  On failure, it returns a negated errno value
 *  to indicate the nature of the error.
 *
 * EAGAIN or EWOULDBLOCK
 *   The socket is marked non-blocking and no connections are present to
 *   be accepted.
 * EOPNOTSUPP
 *   The referenced socket is not of type SOCK_STREAM.
 * EINTR
 *   The system call was interrupted by a signal that was caught before
 *   a valid connection arrived.
 * ECONNABORTED
 *   A connection has been aborted.
 * EINVAL
 *   Socket is not listening for connections.
 * EMFILE
 *   The per-process limit of open file descriptors has been reached.
 * ENFILE
 *   The system maximum for file descriptors has been reached.
 * EFAULT
 *   The addr parameter is not in a writable part of the user address space.
 * ENOBUFS or ENOMEM
 *   Not enough free memory.
 * EPROTO
 *   Protocol error.
 * EPERM
 *   Firewall rules forbid connection.
 *
 ****************************************************************************/

int psock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                 FAR socklen_t *addrlen, FAR struct socket *newsock,
                 int flags)
{
  FAR struct socket_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && newsock != NULL);

  /* May sure that the socket has been opened with socket() */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Socket invalid or not opened\n");
      return -EINVAL;
    }

  /* Is the socket listening for a connection? */

  conn = psock->s_conn;
  if (!_SS_ISLISTENING(conn->s_flags))
    {
      nerr("ERROR: Socket is not listening for a connection.\n");
      return -EINVAL;
    }

  /* Let the address family's accept() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL);
  if (psock->s_sockif->si_accept == NULL)
    {
      return -EOPNOTSUPP;
    }

  net_lock();
  ret = psock->s_sockif->si_accept(psock, addr, addrlen, newsock, flags);
  if (ret >= 0)
    {
      /* Mark the new socket as connected. */

      conn = newsock->s_conn;
      conn->s_flags |= _SF_CONNECTED;
      conn->s_flags &= ~_SF_CLOSED;
      if (flags & SOCK_NONBLOCK)
        {
          conn->s_flags |= _SF_NONBLOCK;
        }
    }
  else
    {
      nerr("ERROR: si_accept failed: %d\n", ret);
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: accept4
 *
 * Description:
 *   The accept4 function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Input Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   flags    The flags used for initialization
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 * EAGAIN or EWOULDBLOCK
 *   The socket is marked non-blocking and no connections are present to
 *   be accepted.
 * EBADF
 *   The descriptor is invalid.
 * ENOTSOCK
 *  The descriptor references a file, not a socket.
 * EOPNOTSUPP
 *   The referenced socket is not of type SOCK_STREAM.
 * EINTR
 *   The system call was interrupted by a signal that was caught before
 *   a valid connection arrived.
 * ECONNABORTED
 *   A connection has been aborted.
 * EINVAL
 *   Socket is not listening for connections.
 * EMFILE
 *   The per-process limit of open file descriptors has been reached.
 * ENFILE
 *   The system maximum for file descriptors has been reached.
 * EFAULT
 *   The addr parameter is not in a writable part of the user address
 *   space.
 * ENOBUFS or ENOMEM
 *   Not enough free memory.
 * EPROTO
 *   Protocol error.
 * EPERM
 *   Firewall rules forbid connection.
 *
 ****************************************************************************/

int accept4(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen,
            int flags)
{
  FAR struct socket *psock = NULL;
  FAR struct socket *newsock;
  int oflags = O_RDWR;
  int errcode;
  int newfd;
  int ret;

  /* accept4() is a cancellation point */

  enter_cancellation_point();

  if (flags & ~(SOCK_NONBLOCK | SOCK_CLOEXEC))
    {
      errcode = EINVAL;
      goto errout;
    }

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &psock);

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  newsock = kmm_zalloc(sizeof(*newsock));
  if (newsock == NULL)
    {
      errcode = ENOMEM;
      goto errout;
    }

  ret = psock_accept(psock, addr, addrlen, newsock, flags);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_alloc;
    }

  /* Allocate a socket descriptor for the new connection now (so that it
   * cannot fail later)
   */

  if (flags & SOCK_CLOEXEC)
    {
      oflags |= O_CLOEXEC;
    }

  if (flags & SOCK_NONBLOCK)
    {
      oflags |= O_NONBLOCK;
    }

  newfd = sockfd_allocate(newsock, oflags);
  if (newfd < 0)
    {
      errcode = ENFILE;
      goto errout_with_psock;
    }

  leave_cancellation_point();
  return newfd;

errout_with_psock:
  psock_close(newsock);

errout_with_alloc:
  kmm_free(newsock);

errout:
  leave_cancellation_point();

  set_errno(errcode);
  return ERROR;
}
