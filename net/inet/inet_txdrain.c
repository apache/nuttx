/****************************************************************************
 * net/inet/inet_txdrain.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/net/net.h>

#include "udp/udp.h"
#include "tcp/tcp.h"
#include "inet/inet.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_txdrain
 *
 * Description:
 *   Wait for any buffered Tx data to be sent from the socket.
 *
 * Parameters:
 *   psock   - Pointer to the socket structure instance
 *   timeout - The relative time when the timeout will occur
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated error value is returned on
 *   any failure.
 *
 ****************************************************************************/

int inet_txdrain(FAR struct socket *psock, unsigned int timeout)
{
  int ret = OK;

  DEBUGASSERT(psock != NULL);

  /* Draining depends on the socket family */

  switch (psock->s_type)
    {
#if defined(NET_TCP_HAVE_STACK)
      case SOCK_STREAM:
        {
          ret = tcp_txdrain(psock, timeout);
        }
        break;
#endif

#if defined(NET_UDP_HAVE_STACK)
      case SOCK_DGRAM:
        {
          ret = udp_txdrain(psock, timeout);
        }
        break;
#endif

     /* Other protocols do no support write buffering */

      default:
        break;
    }

  return ret;
}
