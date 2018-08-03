/****************************************************************************
 * net/netlink/netlink_conn.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "netlink/netlink.h"

#ifdef CONFIG_NET_NETLINK

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all netlink connections. */

static struct netlink_conn_s g_netlink_connections[CONFIG_NET_NETLINK_CONNS];

/* A list of all free netlink connections */

static dq_queue_t g_free_netlink_connections;
static sem_t g_free_sem;

/* A list of all allocated netlink connections */

static dq_queue_t g_active_netlink_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _netlink_semtake() and _netlink_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static void _netlink_semtake(FAR sem_t *sem)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = net_lockedwait(sem)) < 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
    }
}

static void _netlink_semgive(FAR sem_t *sem)
{
  (void)nxsem_post(sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_initialize()
 *
 * Description:
 *   Initialize the User Socket connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void netlink_initialize(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_netlink_connections);
  dq_init(&g_active_netlink_connections);
  nxsem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_NETLINK_CONNS; i++)
    {
      FAR struct netlink_conn_s *conn = &g_netlink_connections[i];

      /* Mark the connection closed and move it to the free list */

      memset(conn, 0, sizeof(*conn));
      dq_addlast(&conn->node, &g_free_netlink_connections);
    }
}

/****************************************************************************
 * Name: netlink_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized netlink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_alloc(void)
{
  FAR struct netlink_conn_s *conn;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _netlink_semtake(&g_free_sem);
  conn = (FAR struct netlink_conn_s *)dq_remfirst(&g_free_netlink_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      memset(conn, 0, sizeof(*conn));

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->node, &g_active_netlink_connections);
    }

  _netlink_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: netlink_free()
 *
 * Description:
 *   Free a netlink connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn)
{
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _netlink_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_netlink_connections);

  /* Reset structure */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_netlink_connections);
  _netlink_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: netlink_nextconn()
 *
 * Description:
 *   Traverse the list of allocated netlink connections
 *
 * Assumptions:
 *   This function is called from netlink device logic.
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_nextconn(FAR struct netlink_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct netlink_conn_s *)g_active_netlink_connections.head;
    }
  else
    {
      return (FAR struct netlink_conn_s *)conn->node.flink;
    }
}

/****************************************************************************
 * Name: netlink_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection for the
 *   provided netlink address
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_active(FAR struct sockaddr_nl *addr)
{
  FAR struct netlink_conn_s *conn = NULL;
#warning "Missing logic for NETLINK active"
  return NULL;
}

#endif /* CONFIG_NET_NETLINK */
