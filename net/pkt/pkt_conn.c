/****************************************************************************
 * net/pkt/pkt_conn.c
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
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>

#include "devif/devif.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define eth_addr_cmp(addr1, addr2) \
  ((addr1[0] == addr2[0]) && (addr1[1] == addr2[1]) && \
   (addr1[2] == addr2[2]) && (addr1[3] == addr2[3]) && \
   (addr1[4] == addr2[4]) && (addr1[5] == addr2[5]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all packet socket connections */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct pkt_conn_s g_pkt_connections[CONFIG_NET_PKT_CONNS];
#endif

/* A list of all free packet socket connections */

static dq_queue_t g_free_pkt_connections;
static sem_t g_free_sem = SEM_INITIALIZER(1);

/* A list of all allocated packet socket connections */

static dq_queue_t g_active_pkt_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _pkt_semtake() and _pkt_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static inline void _pkt_semtake(FAR sem_t *sem)
{
  net_lockedwait_uninterruptible(sem);
}

#define _pkt_semgive(sem) nxsem_post(sem)

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
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;

  for (i = 0; i < CONFIG_NET_PKT_CONNS; i++)
    {
      dq_addlast(&g_pkt_connections[i].sconn.node, &g_free_pkt_connections);
    }
#endif
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
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _pkt_semtake(&g_free_sem);
#ifdef CONFIG_NET_ALLOC_CONNS
  if (dq_peek(&g_free_pkt_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_PKT_CONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_NET_PKT_CONNS; i++)
            {
              dq_addlast(&conn[i].sconn.node, &g_free_pkt_connections);
            }
        }
    }
#endif

  conn = (FAR struct pkt_conn_s *)dq_remfirst(&g_free_pkt_connections);
  if (conn)
    {
      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_pkt_connections);
    }

  _pkt_semgive(&g_free_sem);
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
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _pkt_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_pkt_connections);

  /* Make sure that the connection is marked as uninitialized */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_pkt_connections);
  _pkt_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: pkt_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided Ethernet header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_active(FAR struct eth_hdr_s *buf)
{
  FAR struct pkt_conn_s *conn =
    (FAR struct pkt_conn_s *)g_active_pkt_connections.head;

  while (conn)
    {
      /* FIXME lmac in conn should have been set by pkt_bind() */

      if (eth_addr_cmp(buf->dest, conn->lmac))
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
