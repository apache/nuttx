/****************************************************************************
 * net/socket/listen.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept(). The psock_listen() call
 *   applies only to sockets of type SOCK_STREAM or SOCK_SEQPACKET.
 *
 * Input Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 *   EADDRINUSE
 *     Another socket is already listening on the same port.
 *   EOPNOTSUPP
 *     The socket is not of a type that supports the listen operation.
 *
 ****************************************************************************/

int psock_listen(FAR struct socket *psock, int backlog)
{
  int ret;

  DEBUGASSERT(psock != NULL);

  /* Verify that the sockfd corresponds to a connected SOCK_STREAM */

  if (psock == NULL || psock->s_conn == NULL)
    {
      nerr("ERROR: Invalid or unconnected socket\n");
      return -EINVAL;
    }

  /* Let the address family's listen() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL && psock->s_sockif->si_listen != NULL);
  ret = psock->s_sockif->si_listen(psock, backlog);
  if (ret >= 0)
    {
      FAR struct socket_conn_s *conn = psock->s_conn;
      conn->s_flags |= _SF_LISTENING;
    }
  else
    {
      nerr("ERROR: si_listen failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: listen
 *
 * Description:
 *   To accept connections, a socket is first created with socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with listen(), and then the
 *   connections are accepted with accept(). The listen() call applies only
 *   to sockets of type SOCK_STREAM or SOCK_SEQPACKET.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of the bound socket
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned, and errno is set
 *   appropriately.
 *
 *   EADDRINUSE
 *     Another socket is already listening on the same port.
 *   EBADF
 *     The argument 'sockfd' is not a valid descriptor.
 *   ENOTSOCK
 *     The argument 'sockfd' is not a socket.
 *   EOPNOTSUPP
 *     The socket is not of a type that supports the listen operation.
 *
 ****************************************************************************/

int listen(int sockfd, int backlog)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  FAR struct file *filep;
  int errcode;
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_conn == NULL)
    {
      /* It is not a valid socket description.  Distinguish between the
       * cases where sockfd is a just invalid and when it is a valid file
       * descriptor used in the wrong context.
       */

      if (fs_getfilep(sockfd, &filep) == 0)
        {
          errcode = ENOTSOCK;
        }
      else
        {
          errcode = EBADF;
        }

      _SO_SETERRNO(psock, errcode);
      return ERROR;
    }

  /* The let psock_listen to the work. If psock_listen() fails, it will have
   * set the errno variable.
   */

  ret = psock_listen(psock, backlog);
  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      return ERROR;
    }

  return OK;
}
