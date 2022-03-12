/****************************************************************************
 * net/ieee802154/ieee802154_conn.c
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

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "devif/devif.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections.  Protected via the
 * network lock.
 */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct ieee802154_conn_s
  g_ieee802154_connections[CONFIG_NET_IEEE802154_NCONNS];
#endif

/* A list of all free packet socket connections */

static dq_queue_t g_free_ieee802154_connections;

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_ieee802154_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_conn_initialize
 *
 * Description:
 *   Initialize the IEEE 802.15.4 connection structure allocator.  Called
 *   once and only from ieee802154_initialize().
 *
 * Assumptions:
 *   Called early in the initialization sequence
 *
 ****************************************************************************/

void ieee802154_conn_initialize(void)
{
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;

  for (i = 0; i < CONFIG_NET_IEEE802154_NCONNS; i++)
    {
      /* Link each pre-allocated connection structure into the free list. */

      dq_addlast(&g_ieee802154_connections[i].sconn.node,
                 &g_free_ieee802154_connections);
    }
#endif
}

/****************************************************************************
 * Name: ieee802154_conn_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct ieee802154_conn_s *ieee802154_conn_alloc(void)
{
  FAR struct ieee802154_conn_s *conn;
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* The free list is protected by the network lock. */

  net_lock();
#ifdef CONFIG_NET_ALLOC_CONNS
  if (dq_peek(&g_free_ieee802154_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_IEEE802154_NCONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_NET_IEEE802154_NCONNS; i++)
            {
              dq_addlast(&conn[i].sconn.node,
                         &g_free_ieee802154_connections);
            }
        }
    }
#endif

  conn = (FAR struct ieee802154_conn_s *)
         dq_remfirst(&g_free_ieee802154_connections);
  if (conn)
    {
      dq_addlast(&conn->sconn.node, &g_active_ieee802154_connections);
    }

  net_unlock();
  return conn;
}

/****************************************************************************
 * Name: ieee802154_conn_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void ieee802154_conn_free(FAR struct ieee802154_conn_s *conn)
{
  FAR struct ieee802154_container_s *container;
  FAR struct ieee802154_container_s *next;

  /* The free list is protected by the network lock. */

  DEBUGASSERT(conn->crefs == 0);

  /* Remove the connection from the active list */

  net_lock();
  dq_rem(&conn->sconn.node, &g_active_ieee802154_connections);

  /* Check if there any any frames attached to the container */

  for (container = conn->rxhead; container != NULL; container = next)
    {
      /* Remove the frame from the list */

      next                = container->ic_flink;
      container->ic_flink = NULL;

      /* Free the contained frame data (should be only one in chain) */

      if (container->ic_iob)
        {
          iob_free(container->ic_iob, IOBUSER_NET_SOCK_IEEE802154);
        }

      /* And free the container itself */

      ieee802154_container_free(container);
    }

  /* Enqueue the connection into the active list */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_ieee802154_connections);
  net_unlock();
}

/****************************************************************************
 * Name: ieee802154_conn_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided IEEE 802.15.4 header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct ieee802154_conn_s *
  ieee802154_conn_active(FAR const struct ieee802154_data_ind_s *meta)
{
  FAR struct ieee802154_conn_s *conn;

  DEBUGASSERT(meta != NULL);

  for (conn  = (FAR struct ieee802154_conn_s *)
       g_active_ieee802154_connections.head;
       conn != NULL;
       conn = (FAR struct ieee802154_conn_s *)conn->sconn.node.flink)
    {
      /* Does the destination address match the bound address of the socket.
       *
       * REVISIT: Currently and explicit address must be assigned.  Should we
       * support some moral equivalent to INADDR_ANY?
       */

      if (meta->dest.mode != conn->laddr.s_mode)
        {
          continue;
        }

      if (meta->dest.mode == IEEE802154_ADDRMODE_SHORT &&
          !IEEE802154_SADDRCMP(meta->dest.saddr, conn->laddr.s_saddr))
        {
          continue;
        }

      if (meta->dest.mode == IEEE802154_ADDRMODE_EXTENDED &&
          !IEEE802154_EADDRCMP(meta->dest.saddr, conn->laddr.s_eaddr))
        {
          continue;
        }

      /* Is the socket "connected?" to a remote peer?  If so, check if the
       * source address matches the connected remote address.
       */

      switch (conn->raddr.s_mode)
        {
          case IEEE802154_ADDRMODE_NONE:
            return conn;  /* No.. accept the connection */

          case IEEE802154_ADDRMODE_SHORT:
            if (IEEE802154_SADDRCMP(meta->dest.saddr, conn->raddr.s_saddr))
              {
                return conn;
              }
            break;

          case IEEE802154_ADDRMODE_EXTENDED:
            if (IEEE802154_EADDRCMP(meta->dest.eaddr, conn->raddr.s_eaddr))
              {
                return conn;
              }
            break;

           default:
             nerr("ERROR: Invalid address mode: %u\n", conn->raddr.s_mode);
             return NULL;
        }
    }

  return conn;
}

/****************************************************************************
 * Name: ieee802154_conn_next()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct ieee802154_conn_s *
  ieee802154_conn_next(FAR struct ieee802154_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct ieee802154_conn_s *)
        g_active_ieee802154_connections.head;
    }
  else
    {
      return (FAR struct ieee802154_conn_s *)conn->sconn.node.flink;
    }
}

#endif /* CONFIG_NET_IEEE802154 */
