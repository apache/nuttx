/****************************************************************************
 * net/bluetooth/bluetooth_conn.c
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

#include <netpacket/bluetooth.h>
#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/bluetooth.h>

#include "devif/devif.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections.  Protected via the
 * network lock.
 */

#if CONFIG_NET_BLUETOOTH_PREALLOC_CONNS > 0
static struct bluetooth_conn_s
  g_bluetooth_connections[CONFIG_NET_BLUETOOTH_PREALLOC_CONNS];
#endif

/* A list of all free packet socket connections */

static dq_queue_t g_free_bluetooth_connections;

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_bluetooth_connections;

static const bt_addr_t g_any_addr =
{
  BT_ADDR_ANY
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_conn_initialize
 *
 * Description:
 *   Initialize the Bluetooth connection structure allocator.  Called
 *   once and only from bluetooth_initialize().
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void bluetooth_conn_initialize(void)
{
#if CONFIG_NET_BLUETOOTH_PREALLOC_CONNS > 0
  int i;

  for (i = 0; i < CONFIG_NET_BLUETOOTH_PREALLOC_CONNS; i++)
    {
      /* Link each pre-allocated connection structure into the free list. */

      dq_addlast(&g_bluetooth_connections[i].bc_conn.node,
                 &g_free_bluetooth_connections);
    }
#endif
}

/****************************************************************************
 * Name: bluetooth_conn_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *bluetooth_conn_alloc(void)
{
  FAR struct bluetooth_conn_s *conn;
#if CONFIG_NET_BLUETOOTH_ALLOC_CONNS > 0
  int i;
#endif

  /* The free list is protected by the network lock */

  net_lock();
#if CONFIG_NET_BLUETOOTH_ALLOC_CONNS > 0
  if (dq_peek(&g_active_bluetooth_connections) == NULL)
    {
#if CONFIG_NET_BLUETOOTH_MAX_CONNS > 0
      if (dq_count(&g_active_bluetooth_connections) +
          CONFIG_NET_BLUETOOTH_ALLOC_CONNS >= CONFIG_NET_BLUETOOTH_MAX_CONNS)
        {
          net_unlock();
          return NULL;
        }
#endif

      conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_BLUETOOTH_ALLOC_CONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_NET_BLUETOOTH_ALLOC_CONNS; i++)
            {
              dq_addlast(&conn[i].bc_conn.node,
                         &g_active_bluetooth_connections);
            }
        }
    }
#endif

  conn = (FAR struct bluetooth_conn_s *)
         dq_remfirst(&g_free_bluetooth_connections);
  if (conn)
    {
      /* Mark as unbound */

      conn->bc_proto = BTPROTO_NONE;

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->bc_conn.node, &g_active_bluetooth_connections);
    }

  net_unlock();
  return conn;
}

/****************************************************************************
 * Name: bluetooth_conn_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void bluetooth_conn_free(FAR struct bluetooth_conn_s *conn)
{
  FAR struct bluetooth_container_s *container;
  FAR struct bluetooth_container_s *next;

  /* The free list is protected by the network lock. */

  DEBUGASSERT(conn->bc_crefs == 0);

  /* Remove the connection from the active list */

  net_lock();
  dq_rem(&conn->bc_conn.node, &g_active_bluetooth_connections);

  /* Check if there any any frames attached to the container */

  for (container = conn->bc_rxhead; container != NULL; container = next)
    {
      /* Remove the frame from the list */

      next                = container->bn_flink;
      container->bn_flink = NULL;

      /* Free the contained frame data (should be only one in chain) */

      if (container->bn_iob)
        {
          iob_free(container->bn_iob);
        }

      /* And free the container itself */

      bluetooth_container_free(container);
    }

  /* If this is a preallocated or a batch allocated connection store it in
   * the free connections list. Else free it.
   */

#if CONFIG_NET_BLUETOOTH_ALLOC_CONNS == 1
  if (conn < g_bluetooth_connections || conn >= (g_bluetooth_connections +
      CONFIG_NET_BLUETOOTH_PREALLOC_CONNS))
    {
      kmm_free(conn);
    }
  else
#endif
    {
      memset(conn, 0, sizeof(*conn));
      dq_addlast(&conn->bc_conn.node, &g_free_bluetooth_connections);
    }

  net_unlock();
}

/****************************************************************************
 * Name: bluetooth_conn_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided Bluetooth header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_active(FAR const struct bluetooth_frame_meta_s *meta)
{
  FAR struct bluetooth_conn_s *conn;

  DEBUGASSERT(meta != NULL);

  for (conn =
       (FAR struct bluetooth_conn_s *)g_active_bluetooth_connections.head;
       conn != NULL;
       conn = (FAR struct bluetooth_conn_s *)conn->bc_conn.node.flink)
    {
      /* match protocol and channel first */

      if (meta->bm_proto != conn->bc_proto ||
          meta->bm_channel != conn->bc_channel)
        {
          continue;
        }

      switch (meta->bm_proto)
        {
          /* For BTPROTO_HCI, the socket will not be connected but only
           * bound, thus we match for the device directly
           */

          case BTPROTO_HCI:

            /* TODO: handle when multiple devices supported, need to add ID
             * to meta and conn structures
             */

            goto stop;

            break;

          /* For BTPROTO_L2CAP, the destination address must match the
           * bound address of the socket
           */

          case BTPROTO_L2CAP:
            if ((BLUETOOTH_ADDRCMP(&conn->bc_raddr, &meta->bm_raddr) ||
                 BLUETOOTH_ADDRCMP(&conn->bc_raddr, &g_any_addr)) &&
                (meta->bm_channel == conn->bc_channel))
              {
                goto stop;
              }

            break;
        }
    }

stop:
  return conn;
}

/****************************************************************************
 * Name: bluetooth_conn_next()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct bluetooth_conn_s *
  bluetooth_conn_next(FAR struct bluetooth_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct bluetooth_conn_s *)
        g_active_bluetooth_connections.head;
    }
  else
    {
      return (FAR struct bluetooth_conn_s *)conn->bc_conn.node.flink;
    }
}

#endif /* CONFIG_NET_BLUETOOTH */
