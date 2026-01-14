/****************************************************************************
 * net/icmp/icmp_conn.c
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
#include "utils/utils.h"

#ifdef CONFIG_NET_ICMP_SOCKET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_ICMP_MAX_CONNS
#  define CONFIG_NET_ICMP_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all IPPROTO_ICMP socket connections */

NET_BUFPOOL_DECLARE(g_icmp_connections, sizeof(struct icmp_conn_s),
                    CONFIG_NET_ICMP_PREALLOC_CONNS,
                    CONFIG_NET_ICMP_ALLOC_CONNS, CONFIG_NET_ICMP_MAX_CONNS);

/* A list of all allocated IPPROTO_ICMP socket connections */

static dq_queue_t g_active_icmp_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  /* The free list is protected by a mutex. */

  NET_BUFPOOL_LOCK(g_icmp_connections);

  conn = NET_BUFPOOL_TRYALLOC(g_icmp_connections);
  if (conn != NULL)
    {
      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_icmp_connections);
    }

  NET_BUFPOOL_UNLOCK(g_icmp_connections);

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

  NET_BUFPOOL_LOCK(g_icmp_connections);

  /* free any read-ahead data */

  iob_free_queue(&conn->readahead);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_icmp_connections);
  nxrmutex_destroy(&conn->sconn.s_lock);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_icmp_connections, conn);

  NET_BUFPOOL_UNLOCK(g_icmp_connections);
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
 * Name: icmp_conn_list_lock
 *       icmp_conn_list_unlock
 *
 * Description:
 *   Lock and unlock the ICMP connection list.  This is used to protect the
 *   list of active connections.
 *
 * Assumptions:
 *   This function is called from driver.
 *
 ****************************************************************************/

void icmp_conn_list_lock(void)
{
  NET_BUFPOOL_LOCK(g_icmp_connections);
}

void icmp_conn_list_unlock(void)
{
  NET_BUFPOOL_UNLOCK(g_icmp_connections);
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

  NET_BUFPOOL_LOCK(g_icmp_connections);
  for (conn = icmp_nextconn(NULL); conn != NULL; conn = icmp_nextconn(conn))
    {
      if (conn->id == id && conn->dev == dev)
        {
          return conn;
        }
    }

  NET_BUFPOOL_UNLOCK(g_icmp_connections);
  return conn;
}

/****************************************************************************
 * Name: icmp_foreach
 *
 * Description:
 *   Enumerate each ICMP connection structure. This function will terminate
 *   when either (1) all connection have been enumerated or (2) when a
 *   callback returns any non-zero value.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

int icmp_foreach(icmp_callback_t callback, FAR void *arg)
{
  FAR struct icmp_conn_s *conn;
  int ret = 0;

  NET_BUFPOOL_LOCK(g_icmp_connections);
  if (callback != NULL)
    {
      for (conn = icmp_nextconn(NULL); conn != NULL;
           conn = icmp_nextconn(conn))
        {
          ret = callback(conn, arg);
          if (ret != 0)
            {
              break;
            }
        }
    }

  NET_BUFPOOL_UNLOCK(g_icmp_connections);
  return ret;
}
#endif /* CONFIG_NET_ICMP */
