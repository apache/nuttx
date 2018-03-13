/****************************************************************************
 * net/socket/net_close.c
 *
 *   Copyright (C) 2007-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_close
 *
 * Description:
 *   Performs the close operation on a socket instance
 *
 * Input Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_close(FAR struct socket *psock)
{
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* We perform the close operation only if this is the last count on
   * the socket. (actually, I think the socket crefs only takes the values
   * 0 and 1 right now).
   *
   * It is possible for a psock to have no connection, e.g. a TCP socket
   * waiting in accept.
   */

  if (psock->s_crefs <= 1 && psock->s_conn != NULL)
    {
      /* Let the address family's close() method handle the operation */

      DEBUGASSERT(psock->s_sockif != NULL && psock->s_sockif->si_close != NULL);
      ret = psock->s_sockif->si_close(psock);

      /* Was the close successful */

      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then release our reference on the socket structure containing the connection */

  sock_release(psock);
  return OK;
}

/****************************************************************************
 * Name: net_close
 *
 * Description:
 *   Performs the close operation on socket descriptors
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of socket
 *
 * Returned Value:
 *  Returns zero (OK) on success.  On failure, it returns a negated errno
 *  value to indicate the nature of the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int net_close(int sockfd)
{
  return psock_close(sockfd_socket(sockfd));
}

#endif /* CONFIG_NET */
