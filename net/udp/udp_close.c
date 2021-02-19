/****************************************************************************
 * net/udp/udp_close.c
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
