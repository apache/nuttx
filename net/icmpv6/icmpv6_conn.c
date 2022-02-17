/****************************************************************************
 * net/icmpv6/icmpv6_conn.c
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
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6_SOCKET

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all IPPROTO_ICMP socket connections */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct icmpv6_conn_s g_icmpv6_connections[CONFIG_NET_ICMPv6_NCONNS];
#endif

/* A list of all free IPPROTO_ICMP socket connections */

static dq_queue_t g_free_icmpv6_connections;
static sem_t g_free_sem;

/* A list of all allocated IPPROTO_ICMP socket connections */

static dq_queue_t g_active_icmpv6_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_sock_initialize
 *
 * Description:
 *   Initialize the IPPROTO_ICMP socket connection structures.  Called once
 *   and only from the network initialization layer.
 *
 ****************************************************************************/

void icmpv6_sock_initialize(void)
{
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* Initialize the queues */

  dq_init(&g_free_icmpv6_connections);
  dq_init(&g_active_icmpv6_connections);
  nxsem_init(&g_free_sem, 0, 1);

#ifndef CONFIG_NET_ALLOC_CONNS
  for (i = 0; i < CONFIG_NET_ICMPv6_NCONNS; i++)
    {
      /* Move the connection structure to the free list */

      dq_addlast(&g_icmpv6_connections[i].sconn.node,
                 &g_free_icmpv6_connections);
    }
#endif
}

/****************************************************************************
 * Name: icmpv6_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_alloc(void)
{
  FAR struct icmpv6_conn_s *conn = NULL;
  int ret;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  ret = net_lockedwait(&g_free_sem);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_ALLOC_CONNS
      if (dq_peek(&g_active_icmpv6_connections) == NULL)
        {
          conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_ICMPv6_NCONNS);
          if (conn != NULL)
            {
              for (ret = 0; ret < CONFIG_NET_ICMPv6_NCONNS; ret++)
                {
                  dq_addlast(&conn[ret].sconn.node,
                             &g_free_icmpv6_connections);
                }
            }
        }
#endif

      conn = (FAR struct icmpv6_conn_s *)
             dq_remfirst(&g_free_icmpv6_connections);
      if (conn != NULL)
        {
          /* Enqueue the connection into the active list */

          dq_addlast(&conn->sconn.node, &g_active_icmpv6_connections);
        }

      nxsem_post(&g_free_sem);
    }

  return conn;
}

/****************************************************************************
 * Name: icmpv6_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

void icmpv6_free(FAR struct icmpv6_conn_s *conn)
{
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  /* Take the semaphore (perhaps waiting) */

  net_lockedwait_uninterruptible(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_icmpv6_connections);

  /* Clear the connection structure */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_icmpv6_connections);
  nxsem_post(&g_free_sem);
}

/****************************************************************************
 * Name: icmpv6_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_active(uint16_t id)
{
  FAR struct icmpv6_conn_s *conn =
    (FAR struct icmpv6_conn_s *)g_active_icmpv6_connections.head;

  while (conn != NULL)
    {
      /* FIXME lmac in conn should have been set by icmpv6_bind() */

      if (id == conn->id)
        {
          /* Matching connection found.. return a reference to it */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct icmpv6_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: icmpv6_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_nextconn(FAR struct icmpv6_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct icmpv6_conn_s *)g_active_icmpv6_connections.head;
    }
  else
    {
      return (FAR struct icmpv6_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: icmpv6_findconn
 *
 * Description:
 *   Find an ICMPv6 connection structure that is expecting a ICMPv6 ECHO
 *  response with this ID from this device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_findconn(FAR struct net_driver_s *dev,
                                          uint16_t id)
{
  FAR struct icmpv6_conn_s *conn;

  for (conn = icmpv6_nextconn(NULL); conn != NULL;
       conn = icmpv6_nextconn(conn))
    {
      if (conn->id == id && conn->dev == dev && conn->nreqs > 0)
        {
          return conn;
        }
    }

  return conn;
}
#endif /* CONFIG_NET_ICMP */
