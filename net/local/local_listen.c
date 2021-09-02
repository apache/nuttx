/****************************************************************************
 * net/local/local_listen.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_LOCAL_STREAM)

#include <assert.h>
#include <queue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "local/local.h"

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
 *   returned.  See listen() for the set of appropriate error values.
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

  if (backlog > UINT8_MAX)
    {
      backlog = UINT8_MAX;
    }

  server->u.server.lc_backlog = backlog;

  /* Is this the first time since being bound to an address and that
   * listen() was called?  If so, the state should be LOCAL_STATE_BOUND.
   */

  if (server->lc_state == LOCAL_STATE_BOUND)
    {
      /* The connection should not reside in any other list */

      DEBUGASSERT(server->lc_conn.node.flink == NULL);

      /* And change the server state to listing */

      server->lc_state = LOCAL_STATE_LISTENING;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_STREAM */
