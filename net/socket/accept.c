/****************************************************************************
 * net/socket/accept.c
 *
 *   Copyright (C) 2007-2012, 2015-2016 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0 && \
    (defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL_STREAM))

#include <sys/types.h>
#include <sys/socket.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cancelpt.h>
#include <arch/irq.h>

#include "tcp/tcp.h"
#include "local/local.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_accept
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
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *  Returns 0 (OK) on success.  On failure, it returns -1 (ERROR) with the
 *  errno variable set to indicate the nature of the error.
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

int psock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                 FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  int errcode;
  int ret;

  DEBUGASSERT(psock != NULL);

  /* Treat as a cancellation point */

  (void)enter_cancellation_point();

  /* Is the socket a stream? */

  if (psock->s_type != SOCK_STREAM)
    {
      errcode = EOPNOTSUPP;
      goto errout;
    }

  /* Is the socket listening for a connection? */

  if (!_SS_ISLISTENING(psock->s_flags))
    {
      errcode = EINVAL;
      goto errout;
    }

  /* Verify that a valid memory block has been provided to receive the
   * address
   */

  if (addr)
    {
      /* If an address is provided, then the length must also be provided. */

      DEBUGASSERT(addrlen);

      /* A valid length depends on the address domain */

      switch (psock->s_domain)
        {
#ifdef CONFIG_NET_IPv4
        case PF_INET:
          {
            if (*addrlen < sizeof(struct sockaddr_in))
              {
                errcode = EBADF;
                goto errout;
              }
          }
          break;
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
        case PF_INET6:
          {
            if (*addrlen < sizeof(struct sockaddr_in6))
              {
                errcode = EBADF;
                goto errout;
              }
          }
          break;
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_LOCAL_STREAM
        case PF_LOCAL:
          {
            if (*addrlen < sizeof(sa_family_t))
              {
                errcode = EBADF;
                goto errout;
              }
          }
          break;
#endif /* CONFIG_NET_IPv6 */

        default:
          DEBUGPANIC();
          errcode = EINVAL;
          goto errout;
        }
    }

  /* Initialize the socket structure. */

  newsock->s_domain = psock->s_domain;
  newsock->s_type   = SOCK_STREAM;

  /* Perform the correct accept operation for this address domain */

#ifdef CONFIG_NET_LOCAL_STREAM
#ifdef CONFIG_NET_TCP
  if (psock->s_domain == PF_LOCAL)
#endif
    {
      /* Perform the local accept operation (with the network unlocked) */

      ret = psock_local_accept(psock, addr, addrlen, &newsock->s_conn);
      if (ret < 0)
        {
          errcode = -ret;
          goto errout;
        }
    }
#endif /* CONFIG_NET_LOCAL_STREAM */

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL_STREAM
  else
#endif
    {
      /* Perform the local accept operation (with the network locked) */

      net_lock();
      ret = psock_tcp_accept(psock, addr, addrlen, &newsock->s_conn);
      if (ret < 0)
        {
          net_unlock();
          errcode = -ret;
          goto errout;
        }

      /* Begin monitoring for TCP connection events on the newly connected
       * socket
       */

      ret = net_startmonitor(newsock);
      if (ret < 0)
        {
          /* net_startmonitor() can only fail on certain race conditions
           * where the connection was lost just before this function was
           * called.  Undo everything we have done and return a failure.
           */

          net_unlock();
          errcode = -ret;
          goto errout_after_accept;
        }

      net_unlock();
    }
#endif /* CONFIG_NET_TCP */

  /* Mark the new socket as connected. */

  newsock->s_flags |= _SF_CONNECTED;
  newsock->s_flags &= ~_SF_CLOSED;

  leave_cancellation_point();
  return OK;

errout_after_accept:
  psock_close(newsock);

errout:
  set_errno(errcode);
  leave_cancellation_point();
  return ERROR;
}

/****************************************************************************
 * Function: accept
 *
 * Description:
 *   The accept function is used with connection-based socket types
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
 * Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
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

int accept(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  FAR struct socket *newsock;
  int newfd;
  int errcode;
  int ret;

  /* accept() is a cancellation point */

  (void)enter_cancellation_point();

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      /* It is not a valid socket description.  Distinguish between the cases
       * where sockfd is a just valid and when it is a valid file descriptor used
       * in the wrong context.
       */

#if CONFIG_NFILE_DESCRIPTORS > 0
      if ((unsigned int)sockfd < CONFIG_NFILE_DESCRIPTORS)
        {
          errcode = ENOTSOCK;
        }
      else
#endif
        {
          errcode = EBADF;
        }

      goto errout;
    }

  /* Allocate a socket descriptor for the new connection now (so that it
   * cannot fail later)
   */

  newfd = sockfd_allocate(0);
  if (newfd < 0)
    {
      errcode = ENFILE;
      goto errout;
    }

  newsock = sockfd_socket(newfd);
  if (newsock == NULL)
    {
      errcode = ENFILE;
      goto errout_with_socket;
    }

  ret = psock_accept(psock, addr, addrlen, newsock);
  if (ret < 0)
    {
      /* The errno value has already been set */

      sockfd_release(newfd);
      leave_cancellation_point();
      return ERROR;
    }

  leave_cancellation_point();
  return newfd;

errout_with_socket:
  sockfd_release(newfd);

errout:
  set_errno(errcode);
  leave_cancellation_point();
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS && (CONFIG_NET_TCP || CONFIG_NET_LOCAL_STREAM) */
