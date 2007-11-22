/****************************************************************************
 * net/socket.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <debug.h>

#include "net-internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

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

int socket(int domain, int type, int protocol)
{
  FAR struct socket *psock;
  int sockfd;
  int err;

  /* Only PF_INET or PF_INET6 domains supported */

#ifdef CONFIG_NET_IPv6
  if ( domain != PF_INET6)
#else
  if ( domain != PF_INET)
#endif
    {
      err = EAFNOSUPPORT;
      goto errout;
    }

  /* Only SOCK_STREAM and possible SOCK_DRAM are supported */

#if defined(CONFIG_NET_UDP) && defined(CONFIG_NET_TCP)
  if (protocol != 0 || (type != SOCK_STREAM && type != SOCK_DGRAM))
#elif defined(CONFIG_NET_TCP)
  if (protocol != 0 || type != SOCK_STREAM)
#elif defined(CONFIG_NET_UDP)
  if (protocol != 0 || type != SOCK_DGRAM)
#endif
    {
      err = EPROTONOSUPPORT;
      goto errout;
    }

  /* Everything looks good.  Allocate a socket descriptor */

  sockfd = sockfd_allocate();
  if (sockfd < 0)
    {
      err = ENFILE;
      goto errout;
    }

  /* Initialize the socket structure */

  psock = sockfd_socket(sockfd);
  if (!psock)
    {
      err = ENOSYS; /* should not happen */
      goto errout;
    }

  /* Save the protocol type */

  psock->s_type = type;
  psock->s_conn = NULL;

  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   */

  switch (type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        psock->s_conn = uip_tcpalloc();
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        psock->s_conn = uip_udpalloc();
        break;
#endif

      default:
        break;
    }

  /* Did we succesfully allocate some kind of connection structure? */

  if (!psock->s_conn)
    {
      /* Failed to reserve a connection structure */

      sockfd_release(sockfd);
      err = ENFILE;
      goto errout;
    }

  return sockfd;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET */


