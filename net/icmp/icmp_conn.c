/****************************************************************************
 * net/icmp/icmp_conn.c
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
#include "icmp/icmp.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all IPPROTO_ICMP socket connections */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct icmp_conn_s g_icmp_connections[CONFIG_NET_ICMP_NCONNS];
#endif

/* A list of all free IPPROTO_ICMP socket connections */

static dq_queue_t g_free_icmp_connections;
static mutex_t g_free_lock = NXMUTEX_INITIALIZER;

/* A list of all allocated IPPROTO_ICMP socket connections */

static dq_queue_t g_active_icmp_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_sock_initialize
 *
 * Description:
 *   Initialize the IPPROTO_ICMP socket connection structures.  Called once
 *   and only from the network initialization layer.
 *
 ****************************************************************************/

void icmp_sock_initialize(void)
{
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;

  for (i = 0; i < CONFIG_NET_ICMP_NCONNS; i++)
    {
      /* Move the connection structure to the free list */

      dq_addlast(&g_icmp_connections[i].sconn.node,
                 &g_free_icmp_connections);
    }
#endif
}

/****************************************************************************
 * Name: icmp_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

FAR struct icmp_conn_s *icmp_alloc(void)
{
  FAR struct icmp_conn_s *conn = NULL;
  int ret;

  /* The free list is protected by a mutex. */

  ret = nxmutex_lock(&g_free_lock);
  if (ret >= 0)
    {
#ifdef CONFIG_NET_ALLOC_CONNS
      if (dq_peek(&g_free_icmp_connections) == NULL)
        {
          conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_ICMP_NCONNS);
          if (conn != NULL)
            {
              for (ret = 0; ret < CONFIG_NET_ICMP_NCONNS; ret++)
                {
                  dq_addlast(&conn[ret].sconn.node,
                             &g_free_icmp_connections);
                }
            }
        }
#endif

      conn = (FAR struct icmp_conn_s *)dq_remfirst(&g_free_icmp_connections);
      if (conn != NULL)
        {
          /* Enqueue the connection into the active list */

          dq_addlast(&conn->sconn.node, &g_active_icmp_connections);
        }

      nxmutex_unlock(&g_free_lock);
    }

  return conn;
}

/****************************************************************************
 * Name: icmp_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

void icmp_free(FAR struct icmp_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  /* Take the mutex (perhaps waiting) */

  nxmutex_lock(&g_free_lock);

  /* Is this the last reference on the connection?  It might not be if the
   * socket was cloned.
   */

  if (conn->crefs > 1)
    {
      /* No.. just decrement the reference count */

      conn->crefs--;
    }
  else
    {
      /* Remove the connection from the active list */

      dq_rem(&conn->sconn.node, &g_active_icmp_connections);

      /* Clear the connection structure */

      memset(conn, 0, sizeof(*conn));

      /* Free the connection */

      dq_addlast(&conn->sconn.node, &g_free_icmp_connections);
    }

  nxmutex_unlock(&g_free_lock);
}

/****************************************************************************
 * Name: icmp_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmp_conn_s *icmp_active(uint16_t id)
{
  FAR struct icmp_conn_s *conn =
    (FAR struct icmp_conn_s *)g_active_icmp_connections.head;

  while (conn != NULL)
    {
      /* FIXME lmac in conn should have been set by icmp_bind() */

      if (id == conn->id)
        {
          /* Matching connection found.. return a reference to it */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct icmp_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: icmp_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmp_conn_s *icmp_nextconn(FAR struct icmp_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct icmp_conn_s *)g_active_icmp_connections.head;
    }
  else
    {
      return (FAR struct icmp_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: icmp_findconn
 *
 * Description:
 *   Find an ICMP connection structure that is expecting a ICMP ECHO response
 *   with this ID from this device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmp_conn_s *icmp_findconn(FAR struct net_driver_s *dev,
                                      uint16_t id)
{
  FAR struct icmp_conn_s *conn;

  for (conn = icmp_nextconn(NULL); conn != NULL; conn = icmp_nextconn(conn))
    {
      if (conn->id == id && conn->dev == dev && conn->nreqs > 0)
        {
          return conn;
        }
    }

  return conn;
}
#endif /* CONFIG_NET_ICMP */
