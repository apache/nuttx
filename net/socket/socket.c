/****************************************************************************
 * net/socket/socket.c
 *
 *   Copyright (C) 2007-2009, 2012, 2014-2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/udp.h>

#include "socket/socket.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_tcp_alloc
 *
 * Description:
 *   Allocate and attach a TCP connection structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static int psock_tcp_alloc(FAR struct socket *psock)
{
  /* Allocate the TCP connection structure */

  FAR struct tcp_conn_s *conn = tcp_alloc(psock->s_domain);
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: psock_udp_alloc
 *
 * Description:
 *   Allocate and attach a UDP connection structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static int psock_udp_alloc(FAR struct socket *psock)
{
  /* Allocate the UDP connection structure */

  FAR struct udp_conn_s *conn = udp_alloc(psock->s_domain);
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Name: psock_pkt_alloc
 *
 * Description:
 *   Allocate and attach a raw packet connection structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static int psock_pkt_alloc(FAR struct socket *psock)
{
  /* Allocate the packet socket connection structure and save in the new
   * socket instance.
   */

  FAR struct pkt_conn_s *conn = pkt_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Name: psock_local_alloc
 *
 * Description:
 *   Allocate and attach a local, Unix domain connection structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL
static int psock_local_alloc(FAR struct socket *psock)
{
  /* Allocate the local connection structure */

  FAR struct local_conn_s *conn = local_alloc();
  if (!conn)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

 /* Set the reference count on the connection structure.  This reference
  * count will be incremented only if the socket is dup'ed
  */

  DEBUGASSERT(conn->lc_crefs == 0);
  conn->lc_crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}
#endif /* CONFIG_NET_LOCAL */

/****************************************************************************
 * Public Functions
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
#ifdef CONFIG_NET_LOCAL
  bool ipdomain = false;
#endif
  bool dgramok  = false;
  int ret;
  int err;

  /* Only PF_INET, PF_INET6 or PF_PACKET domains supported */

  switch (domain)
    {
#ifdef CONFIG_NET_IPv4
    case PF_INET:
#ifdef CONFIG_NET_LOCAL
      ipdomain = true;
#endif
      dgramok  = true;
      break;
#endif

#ifdef CONFIG_NET_IPv6
    case PF_INET6:
#ifdef CONFIG_NET_LOCAL
      ipdomain = true;
#endif
      dgramok  = true;
      break;
#endif

#ifdef CONFIG_NET_LOCAL
    case PF_LOCAL:
      dgramok = true;
      break;
#endif

#ifdef CONFIG_NET_PKT
    case PF_PACKET:
      break;
#endif

    default:
      err = EAFNOSUPPORT;
      goto errout;
    }

#if !defined(CONFIG_NET_TCP) && !defined(CONFIG_NET_UDP)
  UNUSED(ipdomain);
#endif

  /* Only SOCK_STREAM, SOCK_DGRAM and possible SOCK_RAW are supported */

  switch (type)
    {
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL)
      case SOCK_STREAM:
#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL
        if (ipdomain)
#endif
          {
            if ((protocol != 0 && protocol != IPPROTO_TCP) || !dgramok)
              {
                err = EPROTONOSUPPORT;
                goto errout;
              }
          }
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_LOCAL
#ifdef CONFIG_NET_TCP
        else
#endif
          {
            if (protocol != 0 || !dgramok)
              {
                err = EPROTONOSUPPORT;
                goto errout;
              }
          }
#endif /* CONFIG_NET_LOCAL */

        break;
#endif /* CONFIG_NET_TCP || CONFIG_NET_LOCAL */

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL)
      case SOCK_DGRAM:
#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL
        if (ipdomain)
#endif
          {
            if ((protocol != 0 && protocol != IPPROTO_UDP) || !dgramok)
              {
                err = EPROTONOSUPPORT;
                goto errout;
              }
          }
#endif /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_LOCAL
#ifdef CONFIG_NET_UDP
        else
#endif
          {
            if (protocol != 0 || !dgramok)
              {
                err = EPROTONOSUPPORT;
                goto errout;
              }
          }
#endif /* CONFIG_NET_LOCAL */

        break;
#endif /* CONFIG_NET_UDP || CONFIG_NET_LOCAL */

#ifdef CONFIG_NET_PKT
      case SOCK_RAW:
        if (dgramok)
          {
            err = EPROTONOSUPPORT;
            goto errout;
          }

        break;
#endif

      default:
        err = EPROTONOSUPPORT;
        goto errout;
    }

  /* Everything looks good.  Initialize the socket structure */
  /* Save the protocol type */

  psock->s_domain = domain;
  psock->s_type   = type;
  psock->s_conn   = NULL;
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  psock->s_sndcb  = NULL;
#endif

  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   */

  err = ENOMEM; /* Assume failure to allocate connection instance */
  switch (type)
    {
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_LOCAL)
      case SOCK_STREAM:
        {
#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_LOCAL
          if (ipdomain)
#endif
            {
              /* Allocate and attach the TCP connection structure */

              ret = psock_tcp_alloc(psock);
            }
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_LOCAL
#ifdef CONFIG_NET_TCP
         else
#endif
            {
              /* Allocate and attach the local connection structure */

              ret = psock_local_alloc(psock);
            }
#endif /* CONFIG_NET_LOCAL */

          /* Check for failures to allocate the connection structure. */

          if (ret < 0)
            {
              /* Failed to reserve a connection structure */

              err = -ret;
              goto errout;
            }
        }
        break;
#endif

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_LOCAL)
      case SOCK_DGRAM:
        {
#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_LOCAL
          if (ipdomain)
#endif
            {
              /* Allocate and attach the UDP connection structure */

              ret = psock_udp_alloc(psock);
            }
#endif /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_LOCAL
#ifdef CONFIG_NET_UDP
         else
#endif
            {
              /* Allocate and attach the local connection structure */

              ret = psock_local_alloc(psock);
            }
#endif /* CONFIG_NET_LOCAL */

          /* Check for failures to allocate the connection structure. */

          if (ret < 0)
            {
              /* Failed to reserve a connection structure */

              err = -ret;
              goto errout;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_PKT
      case SOCK_RAW:
        {
          ret = psock_pkt_alloc(FAR struct socket *psock)
          if (ret < 0)
            {
              /* Failed to reserve a connection structure */

              err = -ret;
              goto errout;
            }
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
