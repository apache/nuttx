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
#include <nuttx/signal.h>
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
 * Name: netlink_notify_waiters
 *
 * Description:
 *   Notify all threads waiting for a response.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void netlink_notify_waiters(FAR struct netlink_conn_s *conn)
{
  int ret;
  int i;

  /* Notify every pending thread.  Lock the scheduler while we do this so
   * that there is no thrashing:  All waiters will be restarted, but only
   * the highest priority waiter will get to run and it will receive the
   * response.
   */

  sched_lock();
  for (i = 0; i < CONFIG_NETLINK_MAXPENDING; i++)
    {
      if (conn->waiter[i] > 0)
        {
#ifdef CONFIG_CAN_PASS_STRUCTS
          union sigval value;

          /* Send the CONFIG_NETLINK_SIGNAL signal to the waiter. */

          value.sival_ptr = conn;
          ret = nxsig_queue((int)conn->waiter[i],
                            (int)CONFIG_NETLINK_SIGNAL, value);
#else
          ret = nxsig_queue((int)conn->waiter[i],
                            (int)CONFIG_NETLINK_SIGNAL, conn);
#endif
          if (ret < 0)
            {
              nerr("ERROR: nxsig_queue() failed: %d\n", ret);
              UNUSED(ret);
            }

          conn->waiter[i] = NETLINK_NO_WAITER;
        }
    }

  sched_unlock();
}

/****************************************************************************
 * Name: netlink_add_waiter
 *
 * Description:
 *   Add one more waiter to the list of waiters.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int netlink_add_waiter(FAR struct netlink_conn_s *conn)
{
  int i;

  for (i = 0; i < CONFIG_NETLINK_MAXPENDING; i++)
    {
      if (conn->waiter[i] <= 0)
        {
          conn->waiter[i] = getpid();
          return OK;
        }
    }

  nerr("ERROR:  Too many waiters\n");
  DEBUGPANIC();
  return -ENOSPC;
}

/****************************************************************************
 * Name: netlink_remove_waiter
 *
 * Description:
 *   Remove a waiter to the list of waiters.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int netlink_remove_waiter(FAR struct netlink_conn_s *conn,
                                 pid_t waiter)
{
  int i;

  for (i = 0; i < CONFIG_NETLINK_MAXPENDING; i++)
    {
      if (conn->waiter[i] == waiter)
        {
          conn->waiter[i] = NETLINK_NO_WAITER;
          break;
        }
    }

  return OK;
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
  int i;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _netlink_semtake(&g_free_sem);
  conn = (FAR struct netlink_conn_s *)dq_remfirst(&g_free_netlink_connections);
  if (conn != NULL)
    {
      /* Make sure that the connection is marked as uninitialized */

      memset(conn, 0, sizeof(*conn));

      /* With no waiters */

      for (i = 0; i < CONFIG_NETLINK_MAXPENDING; i++)
        {
          conn->waiter[i] = NETLINK_NO_WAITER;
        }

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

  netlink_notify_waiters(conn);
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
 *   always returned.  This function will block until a response is
 *   received if the pending response list is empty.
 *
 ****************************************************************************/

FAR struct netlink_response_s *
  netlink_get_response(FAR struct socket *psock)
{
  FAR struct netlink_response_s *resp;
  FAR struct netlink_conn_s *conn;
  FAR struct siginfo info;
  unsigned int count;
  sigset_t set;
  irqstate_t flags;
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
      /* Add this task as a waiter */

      ret = netlink_add_waiter(conn);
      if (ret < 0)
        {
          nerr("ERROR: netlink_add_waiter failed: %d\n", ret);
        }

      /* Break any network lock while we wait */

      flags = enter_critical_section();
      ret = net_breaklock(&count);
      if (ret < 0)
        {
          /* net_breaklock() would only fail if we were not the holder of
           * lock.  But we do hold the lock?
           */

          nerr("ERROR: net_breaklock failed: %d\n", ret);
          DEBUGPANIC();
        }

      /* Wait for a response */

      sigemptyset(&set);
      sigaddset(&set, CONFIG_NETLINK_SIGNAL);
      ret = sigwaitinfo(&set, &info);
      if (ret < 0)
        {
          nerr("ERROR: sigwaitinfo() failed: %d\n", ret);
        }

      /* Restore the network lock */

      net_restorelock(count);
      leave_critical_section(flags);
    }

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
 *   Notify a thread until a response be available.  The thread will be
 *   notified via CONFIG_NETLINK_SIGNAL when the response becomes available.
 *
 * Returned Value:
 *   Zero (OK) is returned if the response is already available.  Not signal
 *     will be sent.
 *   One is returned if the notification was successfully setup.
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int netlink_notify_response(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn;
  FAR struct siginfo info;
  sigset_t set;
  int ret = 0;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Check if the response is available */

  net_lock();
  if (((FAR struct netlink_response_s *)sq_peek(&conn->resplist)) == NULL)
    {
      /* No.. Add this task as a waiter */

      ret = netlink_add_waiter(conn);
      if (ret < 0)
        {
          nerr("ERROR: netlink_add_waiter failed: %d\n", ret);
        }
      else
        {
          /* Set up to signal when a response is available */

          sigemptyset(&set);
          sigaddset(&set, CONFIG_NETLINK_SIGNAL);
          ret = sigwaitinfo(&set, &info);
          if (ret < 0)
            {
              nerr("ERROR: sigwaitinfo() failed: %d\n", ret);
            }
          else
            {
              ret = 1;
            }
        }
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: netlink_notify_cancel
 *
 * Description:
 *   Cancel a notification previously created with netlink_notify_response().
 *
 * Returned Value:
 *   Zero (OK) is always returned.
 *
 ****************************************************************************/

int netlink_notify_cancel(FAR struct socket *psock)
{
  FAR struct netlink_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct netlink_conn_s *)psock->s_conn;

  /* Remove this thread as waiter for response notifications for this
   * socket.
   */

  net_lock();
  (void)netlink_remove_waiter(conn, getpid());
  net_unlock();
  return OK;
}

#endif /* CONFIG_NET_NETLINK */
