/****************************************************************************
 * net/local/local_listen.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL_STREAM)

#include <assert.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "local/local.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* A list of all allocated packet socket connections */

dq_queue_t g_local_listeners;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_listen
 *
 * Description:
 *   Listen for a new connection of a SOCK_STREAM Unix domain socket.
 *
 *   This function is called as part of the implementation of listen();
 *
 * Input Parameters:
 *   server  - A reference to the server-side local connection structure
 *   backlog - Maximum number of pending connections.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The network is NOT locked
 *
 ****************************************************************************/

int local_listen(FAR struct local_conn_s *server, int backlog)
{
  net_lock_t state;

  /* Some sanity checks */

  DEBUGASSERT(server);

  if (server->lc_proto != SOCK_STREAM ||
      server->lc_state == LOCAL_STATE_UNBOUND ||
      server->lc_type != LOCAL_TYPE_PATHNAME)
    {
      return -EOPNOTSUPP;
    }

  DEBUGASSERT(server->lc_state == LOCAL_STATE_BOUND ||
              server->lc_state == LOCAL_STATE_LISTENING);

  /* Set the backlog value */

  DEBUGASSERT((unsigned)backlog < 256);
  server->u.server.lc_backlog = backlog;

  /* Is this the first time since being bound to an address and that
   * listen() was called?  If so, the state should be LOCAL_STATE_BOUND.
   */

  if (server->lc_state == LOCAL_STATE_BOUND)
    {
      /* The connection should not reside in any other list */

      DEBUGASSERT(server->lc_node.flink == NULL &&
                  server->lc_node.flink == NULL);

      /* Add the connection structure to the list of listeners */

      state = net_lock();
      dq_addlast(&server->lc_node, &g_local_listeners);
      net_unlock(state);

      /* And change the server state to listing */

      server->lc_state = LOCAL_STATE_LISTENING;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_STREAM */
