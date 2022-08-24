/****************************************************************************
 * net/local/local_connect.c
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

#ifdef CONFIG_NET_LOCAL_STREAM

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
  net_lockedwait_uninterruptible(sem);
}

#define _local_semgive(sem) nxsem_post(sem)

/****************************************************************************
 * Name: local_stream_connect
 *
 * Description:
 *   Find a local connection structure that is the appropriate "server"
 *   connection to be used with the provided "client" connection.
 *
 * Returned Value:
 *   Zero (OK) returned on success; A negated errno value is returned on a
 *   failure.  Possible failures include:
 *
 * Assumptions:
 *   The network is locked on entry, unlocked on return.  This logic is
 *   an integral part of the lock_connect() implementation and was
 *   separated out only to improve readability.
 *
 ****************************************************************************/

static int inline local_stream_connect(FAR struct local_conn_s *client,
                                       FAR struct local_conn_s *server,
                                       bool nonblock)
{
  int ret;
  int sval;

  /* Has server backlog been reached?
   * NOTE: The backlog will be zero if listen() has never been called by the
   * server.
   */

  if (server->lc_state != LOCAL_STATE_LISTENING ||
      server->u.server.lc_pending >= server->u.server.lc_backlog)
    {
      nerr("ERROR: Server is not listening: lc_state=%d\n",
           server->lc_state);
      nerr("   OR: The backlog limit was reached: %d or %d\n",
           server->u.server.lc_pending, server->u.server.lc_backlog);
      return -ECONNREFUSED;
    }

  /* Increment the number of pending server connection s */

  server->u.server.lc_pending++;
  DEBUGASSERT(server->u.server.lc_pending != 0);

  /* Create the FIFOs needed for the connection */

  ret = local_create_fifos(client);
  if (ret < 0)
    {
      nerr("ERROR: Failed to create FIFOs for %s: %d\n",
           client->lc_path, ret);

      return ret;
    }

  /* Open the client-side write-only FIFO.  This should not block and should
   * prevent the server-side from blocking as well.
   */

  ret = local_open_client_tx(client, nonblock);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open write-only FIFOs for %s: %d\n",
           client->lc_path, ret);

      goto errout_with_fifos;
    }

  DEBUGASSERT(client->lc_outfile.f_inode != NULL);

  /* Set the busy "result" before giving the semaphore. */

  client->u.client.lc_result = -EBUSY;
  client->lc_state = LOCAL_STATE_ACCEPT;

  /* Add ourself to the list of waiting connections and notify the server. */

  dq_addlast(&client->u.client.lc_waiter, &server->u.server.lc_waiters);
  local_event_pollnotify(server, POLLIN);

  if (nxsem_get_value(&server->lc_waitsem, &sval) >= 0 && sval < 1)
    {
      _local_semgive(&server->lc_waitsem);
    }

  /* Wait for the server to accept the connections */

  if (!nonblock)
    {
      do
        {
          _local_semtake(&client->lc_waitsem);
          ret = client->u.client.lc_result;
        }
      while (ret == -EBUSY);

      /* Did we successfully connect? */

      if (ret < 0)
        {
          nerr("ERROR: Failed to connect: %d\n", ret);
          goto errout_with_outfd;
        }
    }

  /* Yes.. open the read-only FIFO */

  ret = local_open_client_rx(client, nonblock);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open write-only FIFOs for %s: %d\n",
           client->lc_path, ret);
      goto errout_with_outfd;
    }

  DEBUGASSERT(client->lc_infile.f_inode != NULL);

  nxsem_post(&client->lc_donesem);

  if (!nonblock)
    {
      client->lc_state = LOCAL_STATE_CONNECTED;
      return ret;
    }

  client->lc_state = LOCAL_STATE_CONNECTING;
  return -EINPROGRESS;

errout_with_outfd:
  file_close(&client->lc_outfile);
  client->lc_outfile.f_inode = NULL;

errout_with_fifos:
  local_release_fifos(client);
  client->lc_state = LOCAL_STATE_BOUND;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_generate_instance_id
 *
 * Description:
 *   Generate instance ID for stream
 *
 ****************************************************************************/

int32_t local_generate_instance_id(void)
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
 * Name: psock_local_connect
 *
 * Description:
 *   Find a local connection structure that is the appropriate "server"
 *   connection to be used with the provided "client" connection.
 *
 * Returned Value:
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
  FAR struct local_conn_s *conn = NULL;

  DEBUGASSERT(psock && psock->s_conn);
  client = (FAR struct local_conn_s *)psock->s_conn;

  if (client->lc_state == LOCAL_STATE_ACCEPT ||
      client->lc_state == LOCAL_STATE_CONNECTED)
    {
      return -EISCONN;
    }

  /* Find the matching server connection */

  net_lock();
  while ((conn = local_nextconn(conn)) != NULL)
    {
      /* Slef found, continue */

      if (conn == psock->s_conn)
        {
          continue;
        }

      /* Handle according to the server connection type */

      switch (conn->lc_type)
        {
        case LOCAL_TYPE_UNNAMED:   /* A Unix socket that is not bound to any name */
        case LOCAL_TYPE_ABSTRACT:  /* lc_path is length zero */
          {
#warning Missing logic
            net_unlock();
            return OK;
          }
          break;

        case LOCAL_TYPE_PATHNAME:  /* lc_path holds a null terminated string */
          {
            /* Anything in the listener list should be a stream socket in the
             * listening state
             */

            if (conn->lc_state == LOCAL_STATE_LISTENING &&
                conn->lc_proto == SOCK_STREAM &&
                strncmp(conn->lc_path, unaddr->sun_path, UNIX_PATH_MAX - 1)
                == 0)
              {
                int ret = OK;

                /* Bind the address and protocol */

                client->lc_type  = conn->lc_type;
                client->lc_proto = conn->lc_proto;
                strlcpy(client->lc_path, unaddr->sun_path,
                        sizeof(client->lc_path));
                client->lc_instance_id = local_generate_instance_id();

                /* The client is now bound to an address */

                client->lc_state = LOCAL_STATE_BOUND;

                /* We have to do more for the SOCK_STREAM family */

                if (conn->lc_proto == SOCK_STREAM)
                  {
                    ret =
                      local_stream_connect(client, conn,
                        _SS_ISNONBLOCK(client->lc_conn.s_flags));
                  }

                net_unlock();
                return ret;
              }
          }
          break;

        default:                 /* Bad, memory must be corrupted */
          DEBUGPANIC();          /* PANIC if debug on, else fall through */

        case LOCAL_TYPE_UNTYPED: /* Type is not determined until the socket is bound */
          {
            net_unlock();
            return -EINVAL;
          }
        }
    }

  net_unlock();
  return -EADDRNOTAVAIL;
}

#endif /* CONFIG_NET_LOCAL_STREAM */
