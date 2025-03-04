/****************************************************************************
 * net/can/can_conn.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "utils/utils.h"
#include "can/can.h"

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CAN_MAX_CONNS
#  define CONFIG_CAN_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all NetLink connections. */

NET_BUFPOOL_DECLARE(g_can_connections, sizeof(struct can_conn_s),
                    CONFIG_CAN_PREALLOC_CONNS, CONFIG_CAN_ALLOC_CONNS,
                    CONFIG_CAN_MAX_CONNS);
static mutex_t g_free_lock = NXMUTEX_INITIALIZER;

/* A list of all allocated NetLink connections */

static dq_queue_t g_active_can_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_initialize()
 *
 * Description:
 *   Initialize the User Socket connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void can_initialize(void)
{
}

/****************************************************************************
 * Name: can_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct can_conn_s *can_alloc(void)
{
  FAR struct can_conn_s *conn;

  /* The free list is protected by a a mutex. */

  nxmutex_lock(&g_free_lock);

  conn = NET_BUFPOOL_TRYALLOC(g_can_connections);
  if (conn != NULL)
    {
      /* FIXME SocketCAN default behavior enables loopback */

#ifdef CONFIG_NET_CANPROTO_OPTIONS
      /* By default the filter is configured to catch all,
       * this is done in commented filter code below:
       *
       * struct can_filter_t catchall_filter;
       * filter.can_id = 0;
       * filter.can_mask = 0;
       * conn->filters[0] = catchall_filter;
       *
       * However memset already sets the filter to 0
       * therefore we only have to set the filter count to 1
       */

      conn->filter_count = 1;
#endif

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_can_connections);
    }

  nxmutex_unlock(&g_free_lock);
  return conn;
}

/****************************************************************************
 * Name: can_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void can_free(FAR struct can_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  nxmutex_lock(&g_free_lock);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_can_connections);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_can_connections, conn);

  nxmutex_unlock(&g_free_lock);
}

/****************************************************************************
 * Name: can_nextconn()
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_nextconn(FAR struct can_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct can_conn_s *)g_active_can_connections.head;
    }
  else
    {
      return (FAR struct can_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: can_active()
 *
 * Description:
 *   Traverse the list of NetLink connections that match dev
 *
 * Input Parameters:
 *   dev  - The device to search for.
 *   conn - The current connection; may be NULL to start the search at the
 *          beginning
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_active(FAR struct net_driver_s *dev,
                                  FAR struct can_conn_s *conn)
{
  while ((conn = can_nextconn(conn)) != NULL)
    {
      if (conn->dev == NULL || conn->dev == dev)
        {
          break;
        }
    }

  return conn;
}

#endif /* CONFIG_NET_CAN */
