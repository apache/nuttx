/****************************************************************************
 * net/local/local_conn.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

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
 ****************************************************************************/

FAR struct local_conn_s *local_alloc(void)
{
  FAR struct local_conn_s *conn =
    (FAR struct local_conn_s *)kmm_zalloc(sizeof(struct local_conn_s));

  if (conn != NULL)
    {
      /* Initialize non-zero elements the new connection structure.  Since
       * Since the memory was allocated with kmm_zalloc(), it is not
       * necessary to zerio-ize any structure elements.
       */

#ifdef CONFIG_NET_LOCAL_STREAM
      nxsem_init(&conn->lc_waitsem, 0, 0);
      nxsem_init(&conn->lc_donesem, 0, 0);

#endif

      /* This semaphore is used for sending safely in multithread.
       * Make sure data will not be garbled when multi-thread sends.
       */

      nxmutex_init(&conn->lc_sendlock);

      /* Add the connection structure to the list of listeners */

      net_lock();
      dq_addlast(&conn->lc_conn.node, &g_local_connections);
      net_unlock();
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
#ifdef CONFIG_NET_LOCAL_SCM
  int i;
#endif /* CONFIG_NET_LOCAL_SCM */

  DEBUGASSERT(conn != NULL);

  /* Remove the server from the list of listeners. */

  net_lock();
  dq_rem(&conn->lc_conn.node, &g_local_connections);

#ifdef CONFIG_NET_LOCAL_SCM
  if (local_peerconn(conn) && conn->lc_peer)
    {
      conn->lc_peer->lc_peer = NULL;
      conn->lc_peer = NULL;
    }
#endif /* CONFIG_NET_LOCAL_SCM */

  net_unlock();

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

#ifdef CONFIG_NET_LOCAL_STREAM
  /* Destroy all FIFOs associted with the connection */

  local_release_fifos(conn);
  nxsem_destroy(&conn->lc_waitsem);
  nxsem_destroy(&conn->lc_donesem);
#endif

  /* Destory sem associated with the connection */

  nxmutex_destroy(&conn->lc_sendlock);

  /* And free the connection structure */

  kmm_free(conn);
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
