/****************************************************************************
 * net/pkt/pkt_conn.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>

#include "devif/devif.h"
#include "pkt/pkt.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define eth_addr_cmp(addr1, addr2) \
  ((addr1[0] == addr2[0]) && (addr1[1] == addr2[1]) && \
   (addr1[2] == addr2[2]) && (addr1[3] == addr2[3]) && \
   (addr1[4] == addr2[4]) && (addr1[5] == addr2[5]))

#ifndef CONFIG_NET_PKT_MAX_CONNS
#  define CONFIG_NET_PKT_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections */

NET_BUFPOOL_DECLARE(g_pkt_connections, sizeof(struct pkt_conn_s),
                    CONFIG_NET_PKT_PREALLOC_CONNS,
                    CONFIG_NET_PKT_ALLOC_CONNS, CONFIG_NET_PKT_MAX_CONNS);
static mutex_t g_free_lock = NXMUTEX_INITIALIZER;

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_pkt_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_initialize()
 *
 * Description:
 *   Initialize the packet socket connection structures.  Called once and
 *   only from the network initialization layer.
 *
 ****************************************************************************/

void pkt_initialize(void)
{
}

/****************************************************************************
 * Name: pkt_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_alloc(void)
{
  FAR struct pkt_conn_s *conn;

  /* The free list is protected by a mutex. */

  nxmutex_lock(&g_free_lock);

  conn = NET_BUFPOOL_TRYALLOC(g_pkt_connections);
  if (conn)
    {
      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_pkt_connections);
    }

  nxmutex_unlock(&g_free_lock);
  return conn;
}

/****************************************************************************
 * Name: pkt_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void pkt_free(FAR struct pkt_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  nxmutex_lock(&g_free_lock);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_pkt_connections);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_pkt_connections, conn);

  nxmutex_unlock(&g_free_lock);
}

/****************************************************************************
 * Name: pkt_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided network device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_active(FAR struct net_driver_s *dev)
{
  FAR struct pkt_conn_s *conn =
    (FAR struct pkt_conn_s *)g_active_pkt_connections.head;

  while (conn)
    {
      if (dev->d_ifindex == conn->ifindex)
        {
          /* Matching connection found.. return a reference to it */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct pkt_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: pkt_nextconn()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_nextconn(FAR struct pkt_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct pkt_conn_s *)g_active_pkt_connections.head;
    }
  else
    {
      return (FAR struct pkt_conn_s *)conn->sconn.node.flink;
    }
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
