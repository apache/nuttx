/****************************************************************************
 * net/can/can_conn.c
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
#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "utils/utils.h"
#include "can/can.h"

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all NetLink connections. */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct can_conn_s g_can_connections[CONFIG_CAN_CONNS];
#endif

/* A list of all free NetLink connections */

static dq_queue_t g_free_can_connections;
static sem_t g_free_sem;

/* A list of all allocated NetLink connections */

static dq_queue_t g_active_can_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _can_semtake() and _can_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static void _can_semtake(FAR sem_t *sem)
{
  net_lockedwait_uninterruptible(sem);
}

static void _can_semgive(FAR sem_t *sem)
{
  nxsem_post(sem);
}

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
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* Initialize the queues */

  dq_init(&g_free_can_connections);
  dq_init(&g_active_can_connections);
  nxsem_init(&g_free_sem, 0, 1);

#ifndef CONFIG_NET_ALLOC_CONNS
  for (i = 0; i < CONFIG_CAN_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */

      dq_addlast(&g_can_connections[i].sconn.node, &g_free_can_connections);
    }
#endif
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
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _can_semtake(&g_free_sem);
#ifdef CONFIG_NET_ALLOC_CONNS
  if (dq_peek(&g_free_can_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(*conn) * CONFIG_CAN_CONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_CAN_CONNS; i++)
            {
              dq_addlast(&conn[i].sconn.node, &g_free_can_connections);
            }
        }
    }
#endif

  conn = (FAR struct can_conn_s *)dq_remfirst(&g_free_can_connections);
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

  _can_semgive(&g_free_sem);
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
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _can_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_can_connections);

  /* Reset structure */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_can_connections);
  _can_semgive(&g_free_sem);
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

#endif /* CONFIG_NET_CAN */
