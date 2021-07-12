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
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "local/local.h"

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
#ifdef CONFIG_NET_LOCAL_STREAM
  dq_init(&g_local_listeners);
#endif
}

/****************************************************************************
 * Name: local_nextconn
 *
 * Description:
 *   Traverse the list of listened local connections
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

FAR struct local_conn_s *local_nextconn(FAR struct local_conn_s *conn)
{
#ifdef CONFIG_NET_LOCAL_STREAM
  if (!conn)
    {
      return (FAR struct local_conn_s *)g_local_listeners.head;
    }
  else
    {
      return (FAR struct local_conn_s *)conn->lc_node.flink;
    }
#else
  return NULL;
#endif
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
      /* This semaphore is used for signaling and, hence, should not have
       * priority inheritance enabled.
       */

      nxsem_init(&conn->lc_waitsem, 0, 0);
      nxsem_set_protocol(&conn->lc_waitsem, SEM_PRIO_NONE);
#endif
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

#ifdef CONFIG_NET_LOCAL_STREAM
  /* Destroy all FIFOs associted with the connection */

  local_release_fifos(conn);
  nxsem_destroy(&conn->lc_waitsem);
#endif

  /* And free the connection structure */

  kmm_free(conn);
}

#endif /* CONFIG_NET && CONFIG_NET_LOCAL */
