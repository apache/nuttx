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
#include <nuttx/mutex.h>
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

#if CONFIG_NET_ICMPv6_PREALLOC_CONNS > 0
static struct icmpv6_conn_s
              g_icmpv6_connections[CONFIG_NET_ICMPv6_PREALLOC_CONNS];
#endif

/* A list of all free IPPROTO_ICMP socket connections */

static dq_queue_t g_free_icmpv6_connections;
static mutex_t g_free_lock = NXMUTEX_INITIALIZER;

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
#if CONFIG_NET_ICMPv6_PREALLOC_CONNS > 0
  int i;

  for (i = 0; i < CONFIG_NET_ICMPv6_PREALLOC_CONNS; i++)
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

  /* The free list is protected by a mutex. */

  ret = nxmutex_lock(&g_free_lock);
  if (ret >= 0)
    {
#if CONFIG_NET_ICMPv6_ALLOC_CONNS > 0
      if (dq_peek(&g_active_icmpv6_connections) == NULL)
        {
#if CONFIG_NET_ICMPv6_MAX_CONNS > 0
          if (dq_count(&g_active_icmpv6_connections) +
              CONFIG_NET_ICMPv6_ALLOC_CONNS >= CONFIG_NET_ICMPv6_MAX_CONNS)
            {
              nxmutex_unlock(&g_free_lock);
              return NULL;
            }
#endif

          conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_ICMPv6_ALLOC_CONNS);
          if (conn != NULL)
            {
              for (ret = 0; ret < CONFIG_NET_ICMPv6_ALLOC_CONNS; ret++)
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

      nxmutex_unlock(&g_free_lock);
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
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  /* Take the mutex (perhaps waiting) */

  nxmutex_lock(&g_free_lock);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_icmpv6_connections);

  /* If this is a preallocated or a batch allocated connection store it in
   * the free connections list. Else free it.
   */

#if CONFIG_NET_ICMPv6_ALLOC_CONNS == 1
  if (conn < g_icmpv6_connections || conn >= (g_icmpv6_connections +
      CONFIG_NET_ICMPv6_PREALLOC_CONNS))
    {
      kmm_free(conn);
    }
  else
#endif
    {
      memset(conn, 0, sizeof(*conn));
      dq_addlast(&conn->sconn.node, &g_free_icmpv6_connections);
    }

  nxmutex_unlock(&g_free_lock);
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
