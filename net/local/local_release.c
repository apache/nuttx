/****************************************************************************
 * net/local/local_release.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/net.h>

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
  net_lock_t state;

  /* There should be no references on this structure */

  DEBUGASSERT(conn->lc_crefs == 0);
  state = net_lock();

  /* We should not bet here with states LOCAL_STATE_CLOSED or with
   * LOCAL_STATE_ACCEPT.  Those are internal states that should be atomic
   * with respect to socket operations.
   */

  DEBUGASSERT(conn->lc_state != LOCAL_STATE_CLOSED &&
              conn->lc_state != LOCAL_STATE_ACCEPT);

  /* If the socket is connected (SOCK_STREAM client), then disconnect it */

  if (conn->lc_state == LOCAL_STATE_CONNECTED ||
      conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      FAR struct local_conn_s *server;

      DEBUGASSERT(conn->lc_family == SOCK_STREAM);

      server = conn->u.client.lc_server;
      DEBUGASSERT(server &&
                  (server->lc_state == LOCAL_STATE_LISTENING ||
                   server->lc_state == LOCAL_STATE_CLOSED) &&
                  !dq_empty(&server->u.server.lc_conns));

      /* Remove ourself from the list of connections */

      dq_rem(&conn->lc_node, &server->u.server.lc_conns);

      /* Is the list of pending connections now empty?  Was the connection
       * already closed?
       */

      if (dq_empty(&server->u.server.lc_waiters) &&
          server->lc_state == LOCAL_STATE_CLOSED)
        {
          /* Yes,  free the server connection as well */

          local_free(server);
        }

      /* Now we can free this connection structure */

      local_free(conn);
    }

  /* Is the socket is listening socket (SOCK_STREAM server) */

  else if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      FAR struct local_conn_s *client;
      FAR struct local_conn_s *next;

      DEBUGASSERT(conn->lc_family == SOCK_STREAM);

      /* Are there still clients waiting for a connection to the server? */

      for (client = (FAR struct local_conn_s *)conn->u.server.lc_waiters.head;
           client;
           client = (FAR struct local_conn_s *)dq_next(&client->lc_node))
        {
          client->u.client.lc_result = -ENOTCONN;
          sem_post(&client->lc_waitsem);
          conn->lc_state = LOCAL_STATE_CLOSED;
        }

      conn->u.server.lc_pending = 0;

      /* Disconnect any previous client connections */

      for (client = (FAR struct local_conn_s *)conn->u.server.lc_conns.head;
           client;
           client = next)
        {
           next = (FAR struct local_conn_s *)dq_next(&client->lc_node);
           dq_rem(&client->lc_node, &conn->u.server.lc_conns);
           client->lc_state = LOCAL_STATE_DISCONNECTED;
        }

      /* Remove the server from the list of listeners */

      dq_rem(&conn->lc_node, &g_local_listeners);

      /* Can we free the connection structure now?  We cannot
       * if there are still pending connection requested to
       * be resolved.
       */

      conn->u.server.lc_backlog = 0;
      if (conn->lc_state == LOCAL_STATE_CLOSED)
        {
          local_free(conn);
        }
    }

  /* For the remaining states (LOCAL_STATE_UNBOUND and LOCAL_STATE_UNBOUND),
   * we simply free the connection structure.
   */

  else
    {
      local_free(conn);
    }

  net_unlock(state);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
