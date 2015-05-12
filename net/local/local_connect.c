/****************************************************************************
 * net/local/local_connnect.c
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

#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "utils/utils.h"
#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_generate_instance_id
 ****************************************************************************/

static int32_t local_generate_instance_id(void)
{
  static int32_t g_next_instance_id = 0;
  int32_t id;

  /* Called from local_connect with net_lock held. */

  id = g_next_instance_id++;
  if (g_next_instance_id < 0)
    {
      g_next_instance_id = 0;
    }

  return id;
}

/****************************************************************************
 * Name: _local_semtake() and _local_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static inline void _local_semtake(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

#define _local_semgive(sem) sem_post(sem)

/****************************************************************************
 * Name: local_stream_connect
 *
 * Description:
 *   Find a local connection structure that is the appropriate "server"
 *   connection to be used with the provided "client" connection.
 *
 * Returned Values:
 *   Zero (OK) returned on success; A negated errno value is returned on a
 *   failure.  Possible failures include:
 *
 * Assumptions:
 *   The network is locked on entry, unlocked on return.  This logic is
 *   an integral part of the lock_connect() implementation and was
 *   separated out only to improve readability.
 *
 ****************************************************************************/

int inline local_stream_connect(FAR struct local_conn_s *client,
                                FAR struct local_conn_s *server,
                                bool nonblock, net_lock_t state)
{
  int ret;

  /* Has server backlog been reached?
   * NOTE: The backlog will be zero if listen() has never been called by the
   * server.
   */

  if (server->lc_state != LOCAL_STATE_LISTENING ||
      server->u.server.lc_pending >= server->u.server.lc_backlog)
    {
      net_unlock(state);
      ndbg("ERROR: Server is not listening: lc_state=%d\n",
           server->lc_state);
      ndbg("   OR: The backlog limit was reached: %d or %d\n",
           server->u.server.lc_pending, server->u.server.lc_backlog);
      return -ECONNREFUSED;
    }

  /* Increment the number of pending server connection s*/

  server->u.server.lc_pending++;
  DEBUGASSERT(server->u.server.lc_pending != 0);

  /* Create the FIFOs needed for the connection */

  ret = local_create_fifos(client);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to create FIFOs for %s: %d\n",
           client->lc_path, ret);

      net_unlock(state);
      return ret;
    }

  /* Open the client-side write-only FIFO.  This should not block and should
   * prevent the server-side from blocking as well.
   */

  ret = local_open_client_tx(client, nonblock);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to open write-only FIFOs for %s: %d\n",
           client->lc_path, ret);

      net_unlock(state);
      goto errout_with_fifos;
    }

  DEBUGASSERT(client->lc_outfd >= 0);

  /* Add ourself to the list of waiting connections and notify the server. */

  dq_addlast(&client->lc_node, &server->u.server.lc_waiters);
  client->lc_state = LOCAL_STATE_ACCEPT;
  local_accept_pollnotify(server, POLLIN);
  _local_semgive(&server->lc_waitsem);
  net_unlock(state);

  /* Wait for the server to accept the connections */

  client->u.client.lc_result = -EBUSY;
  do
    {
      _local_semtake(&client->lc_waitsem);
      ret = client->u.client.lc_result;
    }
  while (ret == -EBUSY);

  /* Did we successfully connect? */

  if (ret < 0)
    {
      ndbg("ERROR: Failed to connect: %d\n", ret);
      goto errout_with_outfd;
    }

  /* Yes.. open the read-only FIFO */

  ret = local_open_client_rx(client, nonblock);
  if (ret < 0)
    {
      ndbg("ERROR: Failed to open write-only FIFOs for %s: %d\n",
           client->lc_path, ret);
      goto errout_with_outfd;
    }

  DEBUGASSERT(client->lc_infd >= 0);
  client->lc_state = LOCAL_STATE_CONNECTED;
  return OK;

errout_with_outfd:
  (void)close(client->lc_outfd);
  client->lc_outfd = -1;

errout_with_fifos:
  (void)local_release_fifos(client);
  client->lc_state = LOCAL_STATE_BOUND;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_local_connect
 *
 * Description:
 *   Find a local connection structure that is the appropriate "server"
 *   connection to be used with the provided "client" connection.
 *
 * Returned Values:
 *   Zero (OK) returned on success; A negated errno value is returned on a
 *   failure.  Possible failures include:
 *
 *   EISCONN - The specified socket is connection-mode and is already
 *     connected.
 *   EADDRNOTAVAIL - The specified address is not available from the
 *     local machine.
 *   ECONNREFUSED - The target address was not listening for connections or
 *     refused the connection request because the connection backlog has
 *     been exceeded.
 *
 ****************************************************************************/

int psock_local_connect(FAR struct socket *psock,
                        FAR const struct sockaddr *addr)
{
  FAR struct local_conn_s *client;
  FAR struct sockaddr_un *unaddr = (FAR struct sockaddr_un *)addr;
  FAR struct local_conn_s *conn;
  net_lock_t state;

  DEBUGASSERT(psock && psock->s_conn);
  client = (FAR struct local_conn_s *)psock->s_conn;

  if (client->lc_state == LOCAL_STATE_ACCEPT ||
      client->lc_state == LOCAL_STATE_CONNECTED)
    {
      return -EISCONN;
    }

  /* Find the matching server connection */

  state = net_lock();
  for(conn = (FAR struct local_conn_s *)g_local_listeners.head;
      conn;
      conn = (FAR struct local_conn_s *)dq_next(&conn->lc_node))
    {
      /* Anything in the listener list should be a stream socket in the
       * istening state
       */

      DEBUGASSERT(conn->lc_state == LOCAL_STATE_LISTENING &&
                  conn->lc_proto == SOCK_STREAM);

      /* Handle according to the server connection type */

      switch (conn->lc_type)
        {
        case LOCAL_TYPE_UNNAMED:   /* A Unix socket that is not bound to any name */
        case LOCAL_TYPE_ABSTRACT:  /* lc_path is length zero */
          {
#warning Missing logic
            net_unlock(state);
            return OK;
          }
          break;

        case LOCAL_TYPE_PATHNAME:  /* lc_path holds a null terminated string */
          {
            if (strncmp(conn->lc_path, unaddr->sun_path, UNIX_PATH_MAX-1) == 0)
              {
                int ret = OK;

                /* Bind the address and protocol */

                client->lc_proto = conn->lc_proto;
                strncpy(client->lc_path, unaddr->sun_path, UNIX_PATH_MAX-1);
                client->lc_path[UNIX_PATH_MAX-1] = '\0';
                client->lc_instance_id = local_generate_instance_id();

                /* The client is now bound to an address */

                client->lc_state = LOCAL_STATE_BOUND;

                /* We have to do more for the SOCK_STREAM family */

                if (conn->lc_proto == SOCK_STREAM)
                  {
                    ret = local_stream_connect(client, conn,
                                               _SS_ISNONBLOCK(psock->s_flags),
                                               state);
                  }
                else
                  {
                    net_unlock(state);
                  }

                return ret;
              }
          }
          break;

        default:                 /* Bad, memory must be corrupted */
          DEBUGPANIC();          /* PANIC if debug on, else fall through */

        case LOCAL_TYPE_UNTYPED: /* Type is not determined until the socket is bound */
          {
            net_unlock(state);
            return -EINVAL;
          }
        }
    }

  net_unlock(state);
  return -EADDRNOTAVAIL;
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_STREAM */
