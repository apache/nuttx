/****************************************************************************
 * net/local/local_accept.c
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

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_waitlisten
 ****************************************************************************/

static int local_waitlisten(FAR struct local_conn_s *server)
{
  int ret;

  /* Loop until a connection is requested or we receive a signal */

  while (dq_empty(&server->u.server.lc_waiters))
    {
      /* No.. wait for a connection or a signal */

      ret = net_lockedwait(&server->lc_waitsem);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* There is a client waiting for the connection */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_accept
 *
 * Description:
 *   This function implements accept() for Unix domain sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening Unix domain socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   newconn  The new, accepted  Unix domain connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is NOT locked.
 *
 ****************************************************************************/

int local_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                 FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  FAR struct local_conn_s *server;
  FAR struct local_conn_s *client;
  FAR struct local_conn_s *conn;
  FAR dq_entry_t *waiter;
  int ret;

  /* Some sanity checks */

  DEBUGASSERT(psock && psock->s_conn);
  DEBUGASSERT(newsock && !newsock->s_conn);

  /* Is the socket a stream? */

  if (psock->s_domain != PF_LOCAL || psock->s_type != SOCK_STREAM)
    {
      return -EOPNOTSUPP;
    }

  /* Verify that a valid memory block has been provided to receive the
   * address
   */

  server = (FAR struct local_conn_s *)psock->s_conn;

  if (server->lc_proto != SOCK_STREAM ||
      server->lc_state != LOCAL_STATE_LISTENING ||
      server->lc_type  != LOCAL_TYPE_PATHNAME)
    {
      return -EOPNOTSUPP;
    }

  /* Loop as necessary if we have to wait for a connection */

  for (; ; )
    {
      /* Are there pending connections.  Remove the client from the
       * head of the waiting list.
       */

      waiter = dq_remfirst(&server->u.server.lc_waiters);

      if (waiter)
        {
          client = container_of(waiter, struct local_conn_s,
                                u.client.lc_waiter);

          /* Decrement the number of pending clients */

          DEBUGASSERT(server->u.server.lc_pending > 0);
          server->u.server.lc_pending--;

          /* Create a new connection structure for the server side of the
           * connection.
           */

          conn = local_alloc();
          if (!conn)
            {
              nerr("ERROR:  Failed to allocate new connection structure\n");
              ret = -ENOMEM;
            }
          else
            {
              /* Initialize the new connection structure */

              conn->lc_crefs  = 1;
              conn->lc_proto  = SOCK_STREAM;
              conn->lc_type   = LOCAL_TYPE_PATHNAME;
              conn->lc_state  = LOCAL_STATE_CONNECTED;
              conn->lc_psock  = psock;
#ifdef CONFIG_NET_LOCAL_SCM
              conn->lc_peer   = client;
              client->lc_peer = conn;
#endif /* CONFIG_NET_LOCAL_SCM */

              strncpy(conn->lc_path, client->lc_path, UNIX_PATH_MAX - 1);
              conn->lc_path[UNIX_PATH_MAX - 1] = '\0';
              conn->lc_instance_id = client->lc_instance_id;

              /* Open the server-side write-only FIFO.  This should not
               * block.
               */

              ret = local_open_server_tx(
                      conn, _SS_ISNONBLOCK(conn->lc_conn.s_flags));
              if (ret < 0)
                {
                  nerr("ERROR: Failed to open write-only FIFOs for %s: %d\n",
                     conn->lc_path, ret);
                }
            }

          /* Do we have a connection?  Is the write-side FIFO opened? */

          if (ret == OK)
            {
              DEBUGASSERT(conn->lc_outfile.f_inode != NULL);

              /* Open the server-side read-only FIFO.  This should not
               * block because the client side has already opening it
               * for writing.
               */

              ret = local_open_server_rx(
                      conn, _SS_ISNONBLOCK(conn->lc_conn.s_flags));
              if (ret < 0)
                {
                   nerr("ERROR: Failed to open read-only FIFOs for %s: %d\n",
                        conn->lc_path, ret);
                }
            }

          /* Do we have a connection?  Are the FIFOs opened? */

          if (ret == OK)
            {
              DEBUGASSERT(conn->lc_infile.f_inode != NULL);

              /* Return the address family */

              if (addr != NULL)
                {
                  ret = local_getaddr(client, addr, addrlen);
                }
            }

          if (ret == OK)
            {
              /* Setup the client socket structure */

              newsock->s_domain = psock->s_domain;
              newsock->s_type   = SOCK_STREAM;
              newsock->s_sockif = psock->s_sockif;
              newsock->s_conn   = (FAR void *)conn;
            }

          /* Signal the client with the result of the connection */

          client->u.client.lc_result = ret;
          if (client->lc_state == LOCAL_STATE_CONNECTING)
            {
              client->lc_state = LOCAL_STATE_CONNECTED;
              _SO_SETERRNO(client->lc_psock, ret);
              local_event_pollnotify(client, POLLOUT);
            }

          nxsem_post(&client->lc_waitsem);
          return ret;
        }

      /* No.. then there should be no pending connections */

      DEBUGASSERT(server->u.server.lc_pending == 0);

      /* Was the socket opened non-blocking? */

      if (_SS_ISNONBLOCK(server->lc_conn.s_flags))
        {
          /* Yes.. return EAGAIN */

          return -EAGAIN;
        }

      /* Otherwise, listen for a connection and try again. */

      ret = local_waitlisten(server);
      if (ret < 0)
        {
          return ret;
        }
    }
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL_STREAM */
