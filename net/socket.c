/****************************************************************************
 * net/socket.c
 *
 *   Copyright (C) 2007-2009, 2012, 2014 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/socket.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/pkt.h>

#include "net.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a socket
 *   structure.
 *
 * Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_socket(int domain, int type, int protocol, FAR struct socket *psock)
{
  int err;

  /* Only PF_INET, PF_INET6 or PF_PACKET domains supported */

  if (
#if defined(CONFIG_NET_IPv6)
      domain != PF_INET6
#else
      domain != PF_INET
#endif
#if defined(CONFIG_NET_PKT)
      && domain != PF_PACKET
#endif
     )
    {
      err = EAFNOSUPPORT;
      goto errout;
    }

  /* Only SOCK_STREAM, SOCK_DGRAM and possible SOCK_RAW are supported */

  if (
#if defined(CONFIG_NET_TCP)
        (type == SOCK_STREAM && protocol != 0 && protocol != IPPROTO_TCP) ||
#endif
#if defined(CONFIG_NET_UDP)
        (type == SOCK_DGRAM  && protocol != 0 && protocol != IPPROTO_UDP) ||
#endif
        (
#if defined(CONFIG_NET_TCP)
#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_PKT)
         type != SOCK_STREAM &&
#else
         type != SOCK_STREAM
#endif
#endif
#if defined(CONFIG_NET_UDP)
#if defined(CONFIG_NET_PKT)
         type != SOCK_DGRAM &&
#else
         type != SOCK_DGRAM
#endif
#endif
#if defined(CONFIG_NET_PKT)
         type != SOCK_RAW
#endif
        )
     )
    {
      err = EPROTONOSUPPORT;
      goto errout;
    }

  /* Everything looks good.  Initialize the socket structure */
  /* Save the protocol type */

  psock->s_type = type;
  psock->s_conn = NULL;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  psock->s_sndcb = NULL;
#endif

  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   */

  err = ENOMEM; /* Assume failure to allocate connection instance */
  switch (type)
    {
#ifdef CONFIG_NET_PKT
      case SOCK_RAW:
        {
          /* Allocate the packet socket connection structure and save
           * in the new socket instance.
           */

          FAR struct pkt_conn_s *conn = pkt_alloc();
          if (!conn)
            {
              /* Failed to reserve a connection structure */

              goto errout;
            }

          /* Set the reference count on the connection structure.  This
           * reference count will be increment only if the socket is
           * dup'ed
           */

          DEBUGASSERT(conn->crefs == 0);
          psock->s_conn = conn;
          conn->crefs   = 1;
        }
        break;
#endif

#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
          /* Allocate the TCP connection structure and save in the new
           * socket instance.
           */

          FAR struct tcp_conn_s *conn = tcp_alloc();
          if (!conn)
            {
              /* Failed to reserve a connection structure */

              goto errout; /* With err == ENFILE or ENOMEM */
            }

          /* Set the reference count on the connection structure.  This
           * reference count will be increment only if the socket is
           * dup'ed
           */

          DEBUGASSERT(conn->crefs == 0);
          psock->s_conn = conn;
          conn->crefs   = 1;
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          /* Allocate the UDP connection structure and save in the new
           * socket instance.
           */

          FAR struct udp_conn_s *conn = udp_alloc();
          if (!conn)
            {
              /* Failed to reserve a connection structure */

              goto errout; /* With err == ENFILE or ENOMEM */
            }

          /* Set the reference count on the connection structure.  This
           * reference count will be increment only if the socket is
           * dup'ed
           */

          DEBUGASSERT(conn->crefs == 0);
          psock->s_conn = conn;
          conn->crefs   = 1;
        }
        break;
#endif

      default:
        break;
    }

  return OK;

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a descriptor.
 *
 * Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   A non-negative socket descriptor on success; -1 on error with errno set
 *   appropriately.
 *
 *   EACCES
 *     Permission to create a socket of the specified type and/or protocol
 *     is denied.
 *   EAFNOSUPPORT
 *     The implementation does not support the specified address family.
 *   EINVAL
 *     Unknown protocol, or protocol family not available.
 *   EMFILE
 *     Process file table overflow.
 *   ENFILE
 *     The system limit on the total number of open files has been reached.
 *   ENOBUFS or ENOMEM
 *     Insufficient memory is available. The socket cannot be created until
 *     sufficient resources are freed.
 *   EPROTONOSUPPORT
 *     The protocol type or the specified protocol is not supported within
 *     this domain.
 *
 * Assumptions:
 *
 ****************************************************************************/

int socket(int domain, int type, int protocol)
{
  FAR struct socket *psock;
  int sockfd;
  int ret;

  /* Allocate a socket descriptor */

  sockfd = sockfd_allocate(0);
  if (sockfd < 0)
    {
      set_errno(ENFILE);
      return ERROR;
    }

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);
  if (!psock)
    {
      set_errno(ENOSYS); /* should not happen */
      goto errout;
    }

  /* Initialize the socket structure */

  ret = psock_socket(domain, type, protocol, psock);
  if (ret < 0)
    {
      /* Error already set by psock_socket() */

      goto errout;
    }

  return sockfd;

errout:
  sockfd_release(sockfd);
  return ERROR;
}

#endif /* CONFIG_NET */
