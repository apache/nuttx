/****************************************************************************
 * net/local/local_conn.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "local/local.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A list of all allocated packet socket connections */

static dq_queue_t g_local_listeners;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  while (net_lockedwait(sem) != 0)
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
                                net_lock_t state)
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
      return -ECONNREFUSED;
    }

  server->u.server.lc_pending++;
  DEBUGASSERT(server->u.server.lc_pending != 0);

  /* Add ourself to the list of waiting connections and notify the server. */

  dq_addlast(&client->lc_node, &server->u.server.lc_waiters);
  client->lc_state = LOCAL_STATE_ACCEPT;
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

  /* Was the connection successful? */

  client->lc_state = (ret < 0 ? LOCAL_STATE_BOUND : LOCAL_STATE_CONNECTED);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_initialize
 *
 * Description:
 *   Initialize the local, Unix domain connection structures.  Called once
 *   and only from the common network initialization logic.
 *
 ****************************************************************************/

 void local_initialize(void)
{
  dq_init(&g_local_listeners);
}

/****************************************************************************
 * Name: local_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized Unix domain socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   API
 *
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void)
{
  FAR struct local_conn_s *conn =
    (FAR struct local_conn_s *)kmm_zalloc(sizeof(struct local_conn_s));

  if (conn)
    {
      /* Initialize non-zero elements the new connection structure */

      conn->lc_fd = -1;
      sem_init(&conn->lc_waitsem, 0, 0);
    }

  return conn;
}

/****************************************************************************
 * Name: local_free()
 *
 * Description:
 *   Free a packet Unix domain connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void local_free(FAR struct local_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);

  /* Make sure that the pipe is closed */

  if (conn->lc_fd >= 0)
    {
      close(conn->lc_fd);
    }

  sem_destroy(&conn->lc_waitsem);

  /* And free the connection structure */

  kmm_free(conn);
}

/****************************************************************************
 * Name: local_bind
 *
 * Description:
 *   This function implements the low-level parts of the standard local
 *   bind()operation.
 *
 ****************************************************************************/

int local_bind(FAR struct local_conn_s *conn,
               FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR const struct sockaddr_un *unaddr =
    (FAR const struct sockaddr_un *)addr;
  int namelen;

  DEBUGASSERT(conn && unaddr && unaddr->sun_family == AF_LOCAL &&
              addrlen >= sizeof(sa_family_t));

  /* Save the address family */

  conn->lc_family = unaddr->sun_family;

  /* No determine the type of the Unix domain socket by comparing the size
   * of the address description.
   */

  if (addrlen == sizeof(sa_family_t))
    {
      /* No sun_path... This is an un-named Unix domain socket */

      conn->lc_type = LOCAL_TYPE_UNNAMED;
    }
  else
    {
      namelen = strnlen(unaddr->sun_path, UNIX_PATH_MAX-1);
      if (namelen <= 0)
        {
          /* Zero-length sun_path... This is an abstract Unix domain socket */

          conn->lc_type    = LOCAL_TYPE_ABSTRACT;
          conn->lc_path[0] = '\0';
        }
      else
        {
          /* This is an normal, pathname Unix domain socket */

          conn->lc_type = LOCAL_TYPE_PATHNAME;

          /* Copy the path into the connection structure */

          (void)strncpy(conn->lc_path, unaddr->sun_path, UNIX_PATH_MAX-1);
          conn->lc_path[UNIX_PATH_MAX-1] = '\0';
        }
    }

  conn->lc_state = LOCAL_STATE_BOUND;
  return OK;
}

/****************************************************************************
 * Name: local_connect
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

int local_connect(FAR struct local_conn_s *client,
                  FAR const struct sockaddr *addr)
{
  FAR struct local_conn_s *conn;
  net_lock_t state;

  DEBUGASSERT(client);

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
      /* Skip over connections that that have not yet been bound,
       * are or a different address family, or are of a different type.
       */

      if (conn->lc_state == LOCAL_STATE_UNBOUND ||
          conn->lc_state == LOCAL_STATE_CLOSED ||
          conn->lc_family != client->lc_family ||
          conn->lc_type != client->lc_type)
        {
          continue;
        }

      /* Handle according to the connection type */

      switch (client->lc_type)
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
            if (strncmp(client->lc_path, conn->lc_path, UNIX_PATH_MAX-1) == 0)
              {
                /* We have to do more for the SOCK_STREAM family */

                if (conn->lc_family == SOCK_STREAM)
                  {
                    return local_stream_connect(client, conn, state);
                  }

                net_unlock(state);
                return OK;
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
          client->u.client.lc_result = -ENETUNREACH;
          _local_semgive(&client->lc_waitsem);
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
