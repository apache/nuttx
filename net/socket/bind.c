/****************************************************************************
 * net/socket/bind.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_bind
 *
 * Description:
 *   bind() gives the socket 'psock' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name
 *   to a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 ****************************************************************************/

int psock_bind(FAR struct socket *psock, const struct sockaddr *addr,
               socklen_t addrlen)
{
  int ret = OK;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_conn == NULL)
    {
      return -ENOTSOCK;
    }

  /* Let the address family's connect() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL && psock->s_sockif->si_bind != NULL);
  ret = psock->s_sockif->si_bind(psock, addr, addrlen);

  /* Was the bind successful */

  if (ret >= 0)
    {
      FAR struct socket_conn_s *conn = psock->s_conn;

      /* Mark the socket bound */

      conn->s_flags |= _SF_BOUND;
    }

  return ret;
}

/****************************************************************************
 * Name: bind
 *
 * Description:
 *   bind() gives the socket 'sockfd' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name to
 *   a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EBADF
 *     sockfd is not a valid descriptor.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     sockfd is a descriptor for a file, not a socket.
 *
 ****************************************************************************/

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct socket *psock;
  int ret;

  /* Get the underlying socket structure */

  ret = sockfd_socket(sockfd, &psock);

  /* Then let psock_bind do all of the work */

  if (ret == OK)
    {
      ret = psock_bind(psock, addr, addrlen);
    }

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* CONFIG_NET */
