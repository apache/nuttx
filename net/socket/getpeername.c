/****************************************************************************
 * nuttx/net/socket/getpeername.c
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_getpeername
 *
 * Description:
 *   The psock_getpeername() function retrieves the remote-connected name of
 *   the specified socket, stores this address in the sockaddr structure
 *   pointed to by the 'addr' argument, and stores the length of this address
 *   in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   psock    Socket structure of socket to operate on
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the
 *   error. Possible errno values that may be returned include:
 *
 *   EBADF      - The socket argument is not a valid file descriptor.
 *   ENOTSOCK   - The socket argument does not refer to a socket.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL     - The socket has been shut down.
 *   ENOBUFS    - Insufficient resources were available in the system to
 *                complete the function.
 *
 ****************************************************************************/

int psock_getpeername(FAR struct socket *psock, FAR struct sockaddr *addr,
                      FAR socklen_t *addrlen)
{
  /* Verify that the psock corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Some sanity checking... Shouldn't need this on a buckled up embedded
   * system (?)
   */

  if (addr == NULL || addrlen == NULL || *addrlen <= 0)
    {
      return -EINVAL;
    }

  /* Let the address family's send() method handle the operation */

  DEBUGASSERT(psock->s_sockif != NULL);

  if (psock->s_sockif->si_getpeername == NULL)
    {
      return -EOPNOTSUPP;
    }

  return psock->s_sockif->si_getpeername(psock, addr, addrlen);
}

/****************************************************************************
 * Name: getpeername
 *
 * Description:
 *   The getpeername() function retrieves the remote-connected name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the
 *   error. Possible errno values that may be returned include:
 *
 *   EBADF      - The socket argument is not a valid file descriptor.
 *   ENOTSOCK   - The socket argument does not refer to a socket.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL     - The socket has been shut down.
 *   ENOBUFS    - Insufficient resources were available in the system to
 *                complete the function.
 *
 ****************************************************************************/

int getpeername(int sockfd, FAR struct sockaddr *addr,
                FAR socklen_t *addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  int ret;

  /* Let psock_getpeername() do all of the work */

  ret = psock_getpeername(psock, addr, addrlen);
  if (ret < 0)
    {
      _SO_SETERRNO(psock, -ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_NET */
