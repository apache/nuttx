/****************************************************************************
 * net/udp/udp_close.c
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_close
 *
 * Description:
 *   Break any current UDP connection
 *
 * Input Parameters:
 *   conn - UDP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int udp_close(FAR struct socket *psock)
{
  FAR struct udp_conn_s *conn;
  unsigned int timeout = UINT_MAX;
  int ret;

  /* Interrupts are disabled here to avoid race conditions */

  net_lock();

  conn = (FAR struct udp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

#ifdef CONFIG_NET_SOLINGER
  /* SO_LINGER
   *   Lingers on a close() if data is present. This option controls the
   *   action taken when unsent messages queue on a socket and close() is
   *   performed. If SO_LINGER is set, the system shall block the calling
   *   thread during close() until it can transmit the data or until the
   *   time expires. If SO_LINGER is not specified, and close() is issued,
   *   the system handles the call in a way that allows the calling thread
   *   to continue as quickly as possible. This option takes a linger
   *   structure, as defined in the <sys/socket.h> header, to specify the
   *   state of the option and linger interval.
   */

  if (_SO_GETOPT(psock->s_options, SO_LINGER))
    {
      timeout = _SO_TIMEOUT(psock->s_linger);
    }
#endif

  /* Wait until for the buffered TX data to be sent. */

  UNUSED(timeout);
  ret = udp_txdrain(psock, timeout);
  if (ret < 0)
    {
      /* udp_txdrain may fail, but that won't stop us from closing
       * the socket.
       */

      nerr("ERROR: udp_txdrain() failed: %d\n", ret);
    }

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  /* Free any semi-permanent write buffer callback in place. */

  if (psock->s_sndcb != NULL)
    {
      udp_callback_free(conn->dev, conn, psock->s_sndcb);
      psock->s_sndcb = NULL;
    }
#endif

  /* And free the connection structure */

  conn->crefs = 0;
  udp_free(psock->s_conn);
  net_unlock();
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
