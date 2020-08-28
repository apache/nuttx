/****************************************************************************
 * net/socket/socket.c
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

#include <sys/socket.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "usrsock/usrsock.h"
#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a socket
 *   structure.
 *
 * Input Parameters:
 *   domain   (see sys/socket.h)
 *   type     (see sys/socket.h)
 *   protocol (see sys/socket.h)
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error:
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
 ****************************************************************************/

int psock_socket(int domain, int type, int protocol,
                 FAR struct socket *psock)
{
  FAR const struct sock_intf_s *sockif = NULL;
  int ret;

  /* Initialize the socket structure */

  psock->s_crefs  = 1;
  psock->s_domain = domain;
  psock->s_proto  = protocol;
  psock->s_conn   = NULL;
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) || defined(CONFIG_NET_UDP_WRITE_BUFFERS)
  psock->s_sndcb  = NULL;
#endif

  if (type & SOCK_CLOEXEC)
    {
      psock->s_flags |= _SF_CLOEXEC;
    }

  if (type & SOCK_NONBLOCK)
    {
      psock->s_flags |= _SF_NONBLOCK;
    }

  type            &= SOCK_TYPE_MASK;
  psock->s_type   = type;

#ifdef CONFIG_NET_USRSOCK
  if (domain != PF_LOCAL && domain != PF_UNSPEC)
    {
      /* Handle special setup for USRSOCK sockets (user-space networking
       * stack).
       */

      ret = g_usrsock_sockif.si_setup(psock, protocol);
      psock->s_sockif = &g_usrsock_sockif;
      return ret;
    }
#endif /* CONFIG_NET_USRSOCK */

  /* Get the socket interface */

  sockif = net_sockif(domain, type, protocol);
  if (sockif == NULL)
    {
      nerr("ERROR: socket address family unsupported: %d\n", domain);
      return -EAFNOSUPPORT;
    }

  /* The remaining of the socket initialization depends on the address
   * family.
   */

  DEBUGASSERT(sockif->si_setup != NULL);
  psock->s_sockif = sockif;

  ret = sockif->si_setup(psock, protocol);
  if (ret < 0)
    {
      nerr("ERROR: socket si_setup() failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: socket
 *
 * Description:
 *   socket() creates an endpoint for communication and returns a descriptor.
 *
 * Input Parameters:
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
  int errcode;
  int sockfd;
  int ret;

  /* Allocate a socket descriptor */

  sockfd = sockfd_allocate(0);
  if (sockfd < 0)
    {
      nerr("ERROR: Failed to allocate a socket descriptor\n");
      errcode = ENFILE;
      goto errout;
    }

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);
  if (!psock)
    {
      errcode = ENOSYS; /* should not happen */
      goto errout_with_sockfd;
    }

  /* Initialize the socket structure */

  ret = psock_socket(domain, type, protocol, psock);
  if (ret < 0)
    {
      nerr("ERROR: psock_socket() failed: %d\n", ret);
      errcode = -ret;
      goto errout_with_sockfd;
    }

  /* The socket has been successfully initialized */

  psock->s_flags |= _SF_INITD;
  return sockfd;

errout_with_sockfd:
  sockfd_release(sockfd);

errout:
  set_errno(errcode);
  return ERROR;
}

#endif /* CONFIG_NET */
