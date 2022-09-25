/****************************************************************************
 * net/local/local_release.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL)

#include <errno.h>
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
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
      conn->lc_state == LOCAL_STATE_CONNECTING ||
      conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      DEBUGASSERT(conn->lc_proto == SOCK_STREAM);

      /* Just free the connection structure */
    }

  /* Is the socket is listening socket (SOCK_STREAM server) */

  else if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      FAR struct local_conn_s *client;
      FAR dq_entry_t *waiter;

      DEBUGASSERT(conn->lc_proto == SOCK_STREAM);

      /* Are there still clients waiting for a connection to the server? */

      for (waiter = dq_peek(&conn->u.server.lc_waiters);
           waiter;
           waiter = dq_next(&client->u.client.lc_waiter))
        {
          client = container_of(waiter, struct local_conn_s,
                                u.client.lc_waiter);
          client->u.client.lc_result = -ENOTCONN;
          nxsem_post(&client->lc_waitsem);
          local_event_pollnotify(client, POLLOUT);
        }

      conn->u.server.lc_pending = 0;
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
