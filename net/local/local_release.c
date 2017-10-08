/****************************************************************************
 * net/local/local_release.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <semaphore.h>
#include <errno.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "local/local.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_release
 *
 * Description:
 *   If the local, Unix domain socket is in the connected state, then
 *   disconnect it.  Release the local connection structure in any event
 *
 * Input Parameters:
 *   conn - A reference to local connection structure
 *
 ****************************************************************************/

int local_release(FAR struct local_conn_s *conn)
{

  /* There should be no references on this structure */

  DEBUGASSERT(conn->lc_crefs == 0);
  net_lock();

#ifdef CONFIG_NET_LOCAL_STREAM
  /* We should not bet here with state LOCAL_STATE_ACCEPT.  That is an
   * internal state that should be atomic with respect to socket operations.
   */

  DEBUGASSERT(conn->lc_state != LOCAL_STATE_ACCEPT);

  /* If the socket is connected (SOCK_STREAM client), then disconnect it */

  if (conn->lc_state == LOCAL_STATE_CONNECTED ||
      conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      DEBUGASSERT(conn->lc_proto == SOCK_STREAM);

      /* Just free the connection structure */
    }

  /* Is the socket is listening socket (SOCK_STREAM server) */

  else if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      FAR struct local_conn_s *client;

      DEBUGASSERT(conn->lc_proto == SOCK_STREAM);

      /* Are there still clients waiting for a connection to the server? */

      for (client = (FAR struct local_conn_s *)conn->u.server.lc_waiters.head;
           client;
           client = (FAR struct local_conn_s *)dq_next(&client->lc_node))
        {
          client->u.client.lc_result = -ENOTCONN;
          nxsem_post(&client->lc_waitsem);
        }

      conn->u.server.lc_pending = 0;

      /* Remove the server from the list of listeners. */

      dq_rem(&conn->lc_node, &g_local_listeners);
    }
#endif /* CONFIG_NET_LOCAL_STREAM */

  /* For the remaining states (LOCAL_STATE_UNBOUND and LOCAL_STATE_UNBOUND),
   * we simply free the connection structure.
   */

  /* Free the connection structure */

  local_free(conn);
  net_unlock();
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
