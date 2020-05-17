/****************************************************************************
 * net/usrsock/usrsock_conn.c
 *
 *  Copyright (C) 2015, 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all usrsock connections. */

static struct usrsock_conn_s g_usrsock_connections[CONFIG_NET_USRSOCK_CONNS];

/* A list of all free usrsock connections */

static dq_queue_t g_free_usrsock_connections;
static sem_t g_free_sem;

/* A list of all allocated usrsock connections */

static dq_queue_t g_active_usrsock_connections;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _usrsock_semtake() and _usrsock_semgive()
 *
 * Description:
 *   Take/give semaphore
 *
 ****************************************************************************/

static void _usrsock_semtake(FAR sem_t *sem)
{
  net_lockedwait_uninterruptible(sem);
}

static void _usrsock_semgive(FAR sem_t *sem)
{
  nxsem_post(sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized usrsock connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_alloc(void)
{
  FAR struct usrsock_conn_s *conn;

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _usrsock_semtake(&g_free_sem);
  conn = (FAR struct usrsock_conn_s *)
    dq_remfirst(&g_free_usrsock_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      memset(conn, 0, sizeof(*conn));
      nxsem_init(&conn->resp.sem, 0, 1);
      conn->dev = NULL;
      conn->usockid = -1;
      conn->state = USRSOCK_CONN_STATE_UNINITIALIZED;
      conn->list = NULL;
      conn->connected = false;

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->node, &g_active_usrsock_connections);
    }

  _usrsock_semgive(&g_free_sem);
  return conn;
}

/****************************************************************************
 * Name: usrsock_free()
 *
 * Description:
 *   Free a usrsock connection structure that is no longer in use. This
 *   should be done by the implementation of close().
 *
 ****************************************************************************/

void usrsock_free(FAR struct usrsock_conn_s *conn)
{
  /* The free list is protected by a semaphore (that behaves like a mutex). */

  DEBUGASSERT(conn->crefs == 0);

  _usrsock_semtake(&g_free_sem);

  /* Remove the connection from the active list */

  dq_rem(&conn->node, &g_active_usrsock_connections);

  /* Reset structure */

  nxsem_destroy(&conn->resp.sem);
  memset(conn, 0, sizeof(*conn));
  conn->dev = NULL;
  conn->usockid = -1;
  conn->state = USRSOCK_CONN_STATE_UNINITIALIZED;
  conn->list = NULL;

  /* Free the connection */

  dq_addlast(&conn->node, &g_free_usrsock_connections);
  _usrsock_semgive(&g_free_sem);
}

/****************************************************************************
 * Name: usrsock_nextconn()
 *
 * Description:
 *   Traverse the list of allocated usrsock connections
 *
 * Assumptions:
 *   This function is called from usrsock device logic.
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_nextconn(FAR struct usrsock_conn_s *conn)
{
  if (!conn)
    {
      return (FAR struct usrsock_conn_s *)g_active_usrsock_connections.head;
    }
  else
    {
      return (FAR struct usrsock_conn_s *)conn->node.flink;
    }
}

/****************************************************************************
 * Name: usrsock_connidx()
 ****************************************************************************/

int usrsock_connidx(FAR struct usrsock_conn_s *conn)
{
  int idx = conn - g_usrsock_connections;

  DEBUGASSERT(idx >= 0);
  DEBUGASSERT(idx < ARRAY_SIZE(g_usrsock_connections));

  return idx;
}

/****************************************************************************
 * Name: usrsock_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection for usrsock
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct usrsock_conn_s *usrsock_active(int16_t usockid)
{
  FAR struct usrsock_conn_s *conn = NULL;

  while ((conn = usrsock_nextconn(conn)) != NULL)
    {
      if (conn->usockid == usockid)
        {
          return conn;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: usrsock_setup_request_callback()
 ****************************************************************************/

int usrsock_setup_request_callback(FAR struct usrsock_conn_s *conn,
                                   FAR struct usrsock_reqstate_s *pstate,
                                   FAR devif_callback_event_t event,
                                   uint16_t flags)
{
  int ret = -EBUSY;

  nxsem_init(&pstate->recvsem, 0, 0);
  nxsem_set_protocol(&pstate->recvsem, SEM_PRIO_NONE);

  pstate->conn   = conn;
  pstate->result = -EAGAIN;
  pstate->completed = false;
  pstate->unlock = false;

  /* Set up the callback in the connection */

  pstate->cb = devif_callback_alloc(NULL, &conn->list);
  if (pstate->cb)
    {
      /* Take a lock since only one outstanding request is allowed */

      if ((flags & USRSOCK_EVENT_REQ_COMPLETE) != 0)
        {
          _usrsock_semtake(&conn->resp.sem);
          pstate->unlock = true;
        }

      /* Set up the connection event handler */

      pstate->cb->flags = flags;
      pstate->cb->priv  = (FAR void *)pstate;
      pstate->cb->event = event;

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usrsock_setup_data_request_callback()
 ****************************************************************************/

int usrsock_setup_data_request_callback(
      FAR struct usrsock_conn_s *conn,
      FAR struct usrsock_data_reqstate_s *pstate,
      FAR devif_callback_event_t event,
      uint16_t flags)
{
  pstate->valuelen = 0;
  pstate->valuelen_nontrunc = 0;
  return usrsock_setup_request_callback(conn, &pstate->reqstate, event,
                                        flags);
}

/****************************************************************************
 * Name: usrsock_teardown_request_callback()
 ****************************************************************************/

void usrsock_teardown_request_callback(FAR struct usrsock_reqstate_s *pstate)
{
  FAR struct usrsock_conn_s *conn = pstate->conn;

  if (pstate->unlock)
    {
      _usrsock_semgive(&conn->resp.sem);
    }

  /* Make sure that no further events are processed */

  devif_conn_callback_free(NULL, pstate->cb, &conn->list);
  nxsem_destroy(&pstate->recvsem);

  pstate->cb = NULL;
}

/****************************************************************************
 * Name: usrsock_setup_datain
 ****************************************************************************/

void usrsock_setup_datain(FAR struct usrsock_conn_s *conn,
                          FAR struct iovec *iov, unsigned int iovcnt)
{
  unsigned int i;

  conn->resp.datain.iov = iov;
  conn->resp.datain.pos = 0;
  conn->resp.datain.total = 0;
  conn->resp.datain.iovcnt = iovcnt;

  for (i = 0; i < iovcnt; i++)
    {
      conn->resp.datain.total += iov[i].iov_len;
    }
}

/****************************************************************************
 * Name: usrsock_initialize()
 *
 * Description:
 *   Initialize the User Socket connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void usrsock_initialize(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_usrsock_connections);
  dq_init(&g_active_usrsock_connections);
  nxsem_init(&g_free_sem, 0, 1);

  for (i = 0; i < CONFIG_NET_USRSOCK_CONNS; i++)
    {
      FAR struct usrsock_conn_s *conn = &g_usrsock_connections[i];

      /* Mark the connection closed and move it to the free list */

      memset(conn, 0, sizeof(*conn));
      conn->dev     = NULL;
      conn->usockid = -1;
      conn->state   = USRSOCK_CONN_STATE_UNINITIALIZED;
      conn->list    = NULL;
      conn->flags   = 0;
      dq_addlast(&conn->node, &g_free_usrsock_connections);
    }

  /* Register /dev/usrsock character device. */

  usrsockdev_register();
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
