/****************************************************************************
 * libs/libc/net/lib_shutdown.c
 *
 *   Copyright (c) 2015, Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
