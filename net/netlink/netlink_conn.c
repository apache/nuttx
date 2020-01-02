/****************************************************************************
 * net/netlink/netlink_conn.c
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netlink.h>

#include "utils/utils.h"
#include "netlink/netlink.h"

#ifdef CONFIG_NET_NETLINK

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all NetLink connections. */

static struct netlink_conn_s g_netlink_connections[CONFIG_NETLINK_CONNS];

/* A list of all free NetLink connections */

static dq_queue_t g_free_netlink_connections;
static sem_t g_free_sem;

/* A list of all allocated NetLink connections */

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
  net_lockedwait_uninterruptible(sem);
}

static void _netlink_semgive(FAR sem_t *sem)
{
  nxsem_post(sem);
}

/****************************************************************************
 * Name: netlink_response_available
 *
 * Description:
 *   Handle a Netlink response available notification.
 *
 * Input Parameters:
 *   Standard work handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void netlink_response_available(FAR void *arg)
{
  /* The entire notifier entry is passed to us.  That is because we are
   * responsible for disposing of the entry via kmm_free() when we are
   * finished with it.
   */

  FAR struct work_notifier_entry_s *notifier =
    (FAR struct work_notifier_entry_s *)arg;
  FAR sem_t *waitsem;

  DEBUGASSERT(notifier != NULL && notifier->info.qualifier != NULL);
  waitsem = (FAR sem_t *)notifier->info.arg;

  /* Free the notifier entry */

  kmm_free(notifier);

  /* wakeup the waiter */

  nxsem_post(waitsem);
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

  for (i = 0; i < CONFIG_NETLINK_CONNS; i++)
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
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_alloc(void)
{
  FAR struct netlink_conn_s *conn;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _netlink_semtake(&g_free_sem);
  conn = (FAR struct netlink_conn_s *)dq_remfirst(&g_free_netlink_connections);
  if (conn != NULL)
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
 *   Free a NetLink connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn)
{
  FAR sq_entry_t *resp;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _netlink_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_netlink_connections);

  /* Free any unclaimed responses */

  while ((resp = sq_remfirst(&conn->resplist)) != NULL)
    {
      kmm_free(resp);
    }

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
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
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
 * Name: netlink_active
 *
 * Description:
 *   Find a connection structure that is the appropriate connection for the
 *   provided NetLink address
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_active(FAR struct sockaddr_nl *addr)
{
  /* This function is used to handle routing of incoming messages to sockets
   * connected to the address.  There is no such use case for NetLink
   * sockets.
   */

  return NULL;
}

/****************************************************************************
 * Name: netlink_add_response
 *
 * Description:
 *   Add response data at the tail of the pending response list.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Input Parameters:
 *   handle - The handle previously provided to the sendto() implementation
 *            for the protocol.  This is an opaque reference to the Netlink
 *            socket state structure.
 *   resp   - The response to the request.  The memory referenced by 'resp'
 *            must have been allocated via kmm_malloc().  It will be freed
 *            using kmm_free() after it has been consumed.
 *
 ****************************************************************************/

void netlink_add_response(NETLINK_HANDLE handle,
                          FAR struct netlink_response_s *resp)
{
  FAR struct socket *psock;
  FAR struct netlink_conn_s *conn;

  psock = (FAR struct socket *)handle;
  DEBUGASSERT(psock != NULL && psock->s_conn != NULL && resp != NULL);

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Add the response to the end of the FIFO list */

  net_lock();
  sq_addlast(&resp->flink, &conn->resplist);

  /* Notify any waiters that a response is available */

  netlink_notifier_signal(conn);
  net_unlock();
}

/****************************************************************************
 * Name: netlink_tryget_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  NULL will be returned if the pending response list is
 *   empty
 *
 ****************************************************************************/

FAR struct netlink_response_s *
  netlink_tryget_response(FAR struct socket *psock)
{
  FAR struct netlink_response_s *resp;
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Return the response at the head of the pending response list (may be
   * NULL).
   */

  net_lock();
  resp = (FAR struct netlink_response_s *)sq_remfirst(&conn->resplist);
  net_unlock();

  return resp;
}

/****************************************************************************
 * Name: netlink_get_response
 *
 * Description:
 *   Return the next response from the head of the pending response list.
 *   Responses are returned one-at-a-time in FIFO order.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Returned Value:
 *   The next response from the head of the pending response list is
 *   returned.  This function will block until a response is received if
 *   the pending response list is empty.  NULL will be returned only in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct netlink_response_s *
  netlink_get_response(FAR struct socket *psock)
{
  FAR struct netlink_response_s *resp;
  FAR struct netlink_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Loop, until a response is received.  A loop is used because in the case
   * of multiple waiters, all waiters will be awakened, but only the highest
   * priority waiter will get the response.
   */

  net_lock();
  while ((resp = netlink_tryget_response(psock)) == NULL)
    {
      sem_t waitsem;

      /* Set up a semaphore to notify us when a response is queued. */

      (void)sem_init(&waitsem, 0, 0);
      (void)nxsem_setprotocol(&waitsem, SEM_PRIO_NONE);

      /* Set up a notifier to post the semaphore when a response is
       * received.
       */

      ret = netlink_notifier_setup(netlink_response_available, conn,
                                   &waitsem);
      if (ret < 0)
        {
          nerr("ERROR: netlink_notifier_setup() failed: %d\n", ret);
        }
      else
        {
          /* Wait for a response to be queued */

          _netlink_semtake(&waitsem);
        }

      /* Clean-up the semaphore */

      sem_destroy(&waitsem);
      netlink_notifier_teardown(conn);

      /* Check for any failures */

      if (ret < 0)
        {
          resp = NULL;
          break;
        }
    }

  net_unlock();
  return resp;
}

/****************************************************************************
 * Name: netlink_check_response
 *
 * Description:
 *   Return true is a response is pending now.
 *
 * Returned Value:
 *   True: A response is available; False; No response is available.
 *
 ****************************************************************************/

bool netlink_check_response(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Check if the response is available.  It is not necessary to lock the
   * network because the sq_peek() is an atomic operation.
   */

  return (sq_peek(&conn->resplist) != NULL);
}

/****************************************************************************
 * Name: netlink_notify_response
 *
 * Description:
 *   Notify a thread when a response is available.  The thread will be
 *   notified via work queue notifier when the response becomes available.
 *
 * Returned Value:
 *   Zero (OK) is returned if the response is already available.  No
 *     notification will be sent.
 *   One is returned if the notification was successfully setup.
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int netlink_notify_response(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn;
  int ret = 0;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Check if the response is available */

  net_lock();
  if (((FAR struct netlink_response_s *)sq_peek(&conn->resplist)) == NULL)
    {
      sem_t waitsem;

      /* No.. Set up a semaphore to notify us when a response is queued. */

      (void)sem_init(&waitsem, 0, 0);
      (void)nxsem_setprotocol(&waitsem, SEM_PRIO_NONE);

      /* Set up a notifier to post the semaphore when a response is
       * received.
       */

      ret = netlink_notifier_setup(netlink_response_available, conn,
                                   &waitsem);
      if (ret < 0)
        {
          nerr("ERROR: netlink_notifier_setup() failed: %d\n", ret);
        }
      else
        {
          /* Wait for a reponse to be queued */

          _netlink_semtake(&waitsem);
        }

      /* Tear-down the notifier and the semaphore */

      netlink_notifier_teardown(conn);
      sem_destroy(&waitsem);
    }

  net_unlock();
  return ret < 0 ? ret : OK;
}

#endif /* CONFIG_NET_NETLINK */
