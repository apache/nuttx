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
#include <debug.h>

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
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of local
 *   Unix sockets, psock_listen() calls this function.  The psock_listen()
 *   call applies only to sockets of type SOCK_STREAM or SOCK_SEQPACKET.
 *
 * Input Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See list() for the set of appropriate error values.
 *
 ****************************************************************************/

int local_listen(FAR struct socket *psock, int backlog)
{
  FAR struct local_conn_s *server;

  /* Verify that the sockfd corresponds to a connected SOCK_STREAM in this
   * address family.
   */

  if (psock->s_domain != PF_LOCAL || psock->s_type != SOCK_STREAM)
    {
      nerr("ERROR: Unsupported socket family=%d or socket type=%d\n",
           psock->s_domain, psock->s_type);
      return -EOPNOTSUPP;
    }

  server = (FAR struct local_conn_s *)psock->s_conn;

  /* Some sanity checks */

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

      net_lock();
      dq_addlast(&server->lc_node, &g_local_listeners);
      net_unlock();

      /* And change the server state to listing */

      server->lc_state = LOCAL_STATE_LISTENING;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_STREAM */
