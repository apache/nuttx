/****************************************************************************
 * libs/libc/net/lib_shutdown.c
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

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: shutdown
 *
 * Description:
 *   The shutdown() function will cause all or part of a full-duplex
 *   connection on the socket associated with the file descriptor socket to
 *   be shut down.
 *
 *   The shutdown() function disables subsequent send and/or receive
 *   operations on a socket, depending on the value of the how argument.
 *
 * Input Parameters:
 *   sockfd - Specifies the file descriptor of the socket.
 *   how    - Specifies the type of shutdown. The values are as follows:
 *
 *     SHUT_RD   - Disables further receive operations.
 *     SHUT_WR   - Disables further send operations.
 *     SHUT_RDWR - Disables further send and receive operations.
 *
 * Returned Value:
 *   Upon successful completion, shutdown() will return 0; otherwise, -1 will
 *   be returned and errno set to indicate the error.
 *
 *     EBADF     - The socket argument is not a valid file descriptor.
 *     EINVAL    - The how argument is invalid.
 *     ENOTCONN  - The socket is not connected.
 *     ENOTSOCK  - The socket argument does not refer to a socket.
 *     ENOBUFS   - Insufficient resources were available in the system to
 *     perform the operation.
 *
 ****************************************************************************/

int shutdown(int sockfd, int how)
{
  /* REVISIT: Not implemented. */

  return OK;
}

#endif /* CONFIG_NET */
