/****************************************************************************
 * net/local/local_conn.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>

#include "local/local.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A list of all allocated packet socket connections */

static dq_queue_t g_local_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_nextconn
 *
 * Description:
 *   Traverse the list of local connections
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *local_nextconn(FAR struct local_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct local_conn_s *)g_local_connections.head;
    }

  return (FAR struct local_conn_s *)conn->lc_conn.node.flink;
}

/****************************************************************************
 * Name: local_findconn
 *
 * Description:
 *   Traverse the connections list to find the local connection
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *
local_findconn(FAR const struct local_conn_s *local_conn,
               FAR const struct sockaddr_un *unaddr)
{
  FAR struct local_conn_s *conn = NULL;

  int index = unaddr->sun_path[0] == '\0' ? 1 : 0;

  while ((conn = local_nextconn(conn)) != NULL)
    {
      if (local_conn->lc_proto == conn->lc_proto &&
          strncmp(conn->lc_path, &unaddr->sun_path[index],
                  UNIX_PATH_MAX - 1) == 0)
        {
          return conn;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: local_peerconn
 *
 * Description:
 *   Traverse the connections list to find the peer
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *local_peerconn(FAR struct local_conn_s *conn)
{
  FAR struct local_conn_s *peer = NULL;

  while ((peer = local_nextconn(peer)) != NULL)
    {
      if (conn->lc_proto == peer->lc_proto && conn != peer &&
          !strncmp(conn->lc_path, peer->lc_path, UNIX_PATH_MAX - 1))
        {
          return peer;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: local_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized Unix domain socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   API
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void)
{
  FAR struct local_conn_s *conn =
    kmm_zalloc(sizeof(struct local_conn_s));

  if (conn != NULL)
    {
      /* Initialize non-zero elements the new connection structure.  Since
       * Since the memory was allocated with kmm_zalloc(), it is not
       * necessary to zerio-ize any structure elements.
       */

      conn->lc_crefs = 1;
      conn->lc_rcvsize = CONFIG_DEV_FIFO_SIZE;

#ifdef CONFIG_NET_LOCAL_STREAM
      nxsem_init(&conn->lc_waitsem, 0, 0);
#endif

      /* This semaphore is used for sending safely in multithread.
       * Make sure data will not be garbled when multi-thread sends.
       */

      nxmutex_init(&conn->lc_sendlock);
      nxmutex_init(&conn->lc_polllock);

#ifdef CONFIG_NET_LOCAL_SCM
      conn->lc_cred.pid = nxsched_getpid();
      conn->lc_cred.uid = getuid();
      conn->lc_cred.gid = getgid();
#endif

      /* Add the connection structure to the list of listeners */

      dq_addlast(&conn->lc_conn.node, &g_local_connections);
    }

  return conn;
}

/****************************************************************************
 * Name: local_alloc_accept
 *
 * Description:
 *    Called when a client calls connect and can find the appropriate
 *    connection in LISTEN. In that case, this function will create
 *    a new connection and initialize it.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

int local_alloc_accept(FAR struct local_conn_s *server,
                       FAR struct local_conn_s *client,
                       FAR struct local_conn_s **accept)
{
  FAR struct local_conn_s *conn;
  int ret;

  /* Create a new connection structure for the server side of the
   * connection.
   */

  conn = local_alloc();
  if (conn == NULL)
    {
      nerr("ERROR:  Failed to allocate new connection structure\n");
      return -ENOMEM;
    }

  conn->lc_proto  = SOCK_STREAM;
  conn->lc_type   = LOCAL_TYPE_PATHNAME;
  conn->lc_state  = LOCAL_STATE_CONNECTED;
  conn->lc_peer   = client;
  client->lc_peer = conn;

  strlcpy(conn->lc_path, server->lc_path, sizeof(conn->lc_path));
  conn->lc_instance_id = client->lc_instance_id;

  /* Create the FIFOs needed for the connection */

  ret = local_create_fifos(conn, server->lc_rcvsize, client->lc_rcvsize);
  if (ret < 0)
    {
      nerr("ERROR: Failed to create FIFOs for %s: %d\n",
           client->lc_path, ret);
      goto err;
    }

  /* Open the server-side write-only FIFO.  This should not
   * block.
   */

  ret = local_open_server_tx(conn, false);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open write-only FIFOs for %s: %d\n",
           conn->lc_path, ret);
      goto errout_with_fifos;
    }

  /* Do we have a connection?  Is the write-side FIFO opened? */

  DEBUGASSERT(conn->lc_outfile.f_inode != NULL);

  /* Open the server-side read-only FIFO.  This should not
   * block because the client side has already opening it
   * for writing.
   */

  ret = local_open_server_rx(conn, false);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open read-only FIFOs for %s: %d\n",
           conn->lc_path, ret);
      goto errout_with_fifos;
    }

  /* Do we have a connection?  Are the FIFOs opened? */

  DEBUGASSERT(conn->lc_infile.f_inode != NULL);
  *accept = conn;
  return OK;

errout_with_fifos:
  local_release_fifos(conn);

err:
  local_free(conn);
  return ret;
}

/****************************************************************************
 * Name: local_free()
 *
 * Description:
 *   Free a packet Unix domain connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

void local_free(FAR struct local_conn_s *conn)
{
#ifdef CONFIG_NET_LOCAL_SCM
  int i;
#endif /* CONFIG_NET_LOCAL_SCM */

  DEBUGASSERT(conn != NULL);

  /* Remove the server from the list of listeners. */

  dq_rem(&conn->lc_conn.node, &g_local_connections);

  if (conn->lc_peer)
    {
      conn->lc_peer->lc_peer = NULL;
      conn->lc_peer = NULL;
    }

  /* Make sure that the read-only FIFO is closed */

  if (conn->lc_infile.f_inode != NULL)
    {
      file_close(&conn->lc_infile);
      conn->lc_infile.f_inode = NULL;
    }

  /* Make sure that the write-only FIFO is closed */

  if (conn->lc_outfile.f_inode != NULL)
    {
      file_close(&conn->lc_outfile);
      conn->lc_outfile.f_inode = NULL;
    }

#ifdef CONFIG_NET_LOCAL_SCM
  /* Free the pending control file pointer */

  for (i = 0; i < conn->lc_cfpcount; i++)
    {
      if (conn->lc_cfps[i])
        {
          file_close(conn->lc_cfps[i]);
          kmm_free(conn->lc_cfps[i]);
          conn->lc_cfps[i] = NULL;
        }
    }
#endif /* CONFIG_NET_LOCAL_SCM */

  /* Destroy all FIFOs associted with the connection */

  local_release_fifos(conn);
#ifdef CONFIG_NET_LOCAL_STREAM
  nxsem_destroy(&conn->lc_waitsem);
#endif

  /* Destory sem associated with the connection */

  nxmutex_destroy(&conn->lc_sendlock);
  nxmutex_destroy(&conn->lc_polllock);

  /* And free the connection structure */

  kmm_free(conn);
}

/****************************************************************************
 * Name: local_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void local_addref(FAR struct local_conn_s *conn)
{
  DEBUGASSERT(conn->lc_crefs < 255);
  conn->lc_crefs++;
}

/****************************************************************************
 * Name: local_subref
 *
 * Description:
 *   Subtract the reference count on the underlying connection structure.
 *
 * Input Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void local_subref(FAR struct local_conn_s *conn)
{
  DEBUGASSERT(conn->lc_crefs > 0 && conn->lc_crefs < 255);

  conn->lc_crefs--;
  if (conn->lc_crefs == 0)
    {
      local_release(conn);
    }
}
