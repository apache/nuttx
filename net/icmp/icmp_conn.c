/****************************************************************************
 * net/icmp/icmp_conn.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/semaphore.h>
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

static struct icmp_conn_s g_icmp_connections[CONFIG_NET_ICMP_NCONNS];

/* A list of all free IPPROTO_ICMP socket connections */

static dq_queue_t g_free_icmp_connections;
static sem_t g_free_sem;

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
  int i;

  /* Initialize the queues */

  dq_init(&g_free_icmp_connections);
  dq_init(&g_active_icmp_connections);
  nxsem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_ICMP_NCONNS; i++)
    {
      /* Move the connection structure to the free list */

      dq_addlast(&g_icmp_connections[i].node, &g_free_icmp_connections);
    }
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

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  ret = net_lockedwait(&g_free_sem);
  if (ret >= 0)
    {
      conn = (FAR struct icmp_conn_s *)dq_remfirst(&g_free_icmp_connections);
      if (conn != NULL)
        {
          /* Clear the connection structure */

          memset(conn, 0, sizeof(struct icmp_conn_s));

          /* Enqueue the connection into the active list */

          dq_addlast(&conn->node, &g_active_icmp_connections);
        }

      nxsem_post(&g_free_sem);
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
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  /* Take the semaphore (perhaps waiting) */

  net_lockedwait_uninterruptible(&g_free_sem);

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

      dq_rem(&conn->node, &g_active_icmp_connections);

      /* Free the connection */

      dq_addlast(&conn->node, &g_free_icmp_connections);
      nxsem_post(&g_free_sem);
    }
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

      conn = (FAR struct icmp_conn_s *)conn->node.flink;
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
      return (FAR struct icmp_conn_s *)conn->node.flink;
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
