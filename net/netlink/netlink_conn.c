/****************************************************************************
 * net/netlink/netlink_conn.c
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

#ifndef CONFIG_NET_ALLOC_CONNS
static struct netlink_conn_s g_netlink_connections[CONFIG_NETLINK_CONNS];
#endif

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
  DEBUGASSERT(arg != NULL);

  /* wakeup the waiter */

  _netlink_semgive(arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_initialize()
 *
 * Description:
 *   Initialize the NetLink connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void netlink_initialize(void)
{
#ifndef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* Initialize the queues */

  dq_init(&g_free_netlink_connections);
  dq_init(&g_active_netlink_connections);
  nxsem_init(&g_free_sem, 0, 1);

#ifndef CONFIG_NET_ALLOC_CONNS
  for (i = 0; i < CONFIG_NETLINK_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */

      dq_addlast(&g_netlink_connections[i].sconn.node,
                 &g_free_netlink_connections);
    }
#endif
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
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _netlink_semtake(&g_free_sem);
#ifdef CONFIG_NET_ALLOC_CONNS
  if (dq_peek(&g_free_netlink_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(*conn) * CONFIG_NETLINK_CONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_NETLINK_CONNS; i++)
            {
              dq_addlast(&conn[i].sconn.node, &g_free_netlink_connections);
            }
        }
    }
#endif

  conn = (FAR struct netlink_conn_s *)
           dq_remfirst(&g_free_netlink_connections);
  if (conn != NULL)
    {
      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_netlink_connections);
    }

  _netlink_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: netlink_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn)
{
  FAR sq_entry_t *resp;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _netlink_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_netlink_connections);

  /* Free any unclaimed responses */

  while ((resp = sq_remfirst(&conn->resplist)) != NULL)
    {
      kmm_free(resp);
    }

  /* Reset structure */

  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_netlink_connections);
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
      return (FAR struct netlink_conn_s *)conn->sconn.node.flink;
    }
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
  FAR struct netlink_conn_s *conn;

  conn = handle;
  DEBUGASSERT(conn != NULL && resp != NULL);

  /* Add the response to the end of the FIFO list */

  net_lock();
  sq_addlast(&resp->flink, &conn->resplist);

  /* Notify any waiters that a response is available */

  netlink_notifier_signal(conn);
  net_unlock();
}

/****************************************************************************
 * Name: netlink_add_broadcast
 *
 * Description:
 *   Add broadcast data to all interested netlink connections.
 *
 *   Note:  The network will be momentarily locked to support exclusive
 *   access to the pending response list.
 *
 * Input Parameters:
 *   group - The broadcast group index.
 *   data  - The broadcast data.  The memory referenced by 'data'
 *           must have been allocated via kmm_malloc().  It will be freed
 *           using kmm_free() after it has been consumed.
 *
 ****************************************************************************/

void netlink_add_broadcast(int group, FAR struct netlink_response_s *data)
{
  FAR struct netlink_conn_s *conn = NULL;
  int first = 1;

  DEBUGASSERT(data != NULL);

  net_lock();

  while ((conn = netlink_nextconn(conn)) != NULL)
    {
      if ((conn->groups & (1 << (group - 1))) == 0)
        {
          continue;
        }

      /* Duplicate the package except the first loop */

      if (!first)
        {
          FAR struct netlink_response_s *tmp;
          size_t len;

          len = sizeof(sq_entry_t) + data->msg.nlmsg_len;
          tmp = kmm_malloc(len);
          if (tmp == NULL)
            {
              break;
            }

          memcpy(tmp, data, len);
          data = tmp;
        }

      first = 0;

      /* Add the response to the end of the FIFO list */

      sq_addlast(&data->flink, &conn->resplist);

      /* Notify any waiters that a response is available */

      netlink_notifier_signal(conn);
    }

  net_unlock();

  /* Drop the package if nobody is interested in */

  if (first)
    {
      kmm_free(data);
    }
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
netlink_tryget_response(FAR struct netlink_conn_s *conn)
{
  FAR struct netlink_response_s *resp;

  DEBUGASSERT(conn != NULL);

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
netlink_get_response(FAR struct netlink_conn_s *conn)
{
  FAR struct netlink_response_s *resp;
  int ret;

  DEBUGASSERT(conn != NULL);

  /* Loop, until a response is received.  A loop is used because in the case
   * of multiple waiters, all waiters will be awakened, but only the highest
   * priority waiter will get the response.
   */

  net_lock();
  while ((resp = netlink_tryget_response(conn)) == NULL)
    {
      sem_t waitsem;

      /* Set up a semaphore to notify us when a response is queued. */

      nxsem_init(&waitsem, 0, 0);
      nxsem_set_protocol(&waitsem, SEM_PRIO_NONE);

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

      nxsem_destroy(&waitsem);
      netlink_notifier_teardown(conn);

      /* Check for any failures */

      if (ret < 0)
        {
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

bool netlink_check_response(FAR struct netlink_conn_s *conn)
{
  DEBUGASSERT(conn != NULL);

  /* Check if the response is available.  It is not necessary to lock the
   * network because the sq_peek() is an atomic operation.
   */

  return (sq_peek(&conn->resplist) != NULL);
}

#endif /* CONFIG_NET_NETLINK */
