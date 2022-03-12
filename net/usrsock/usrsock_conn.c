/****************************************************************************
 * net/usrsock/usrsock_conn.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all usrsock connections. */

#ifndef CONFIG_NET_ALLOC_CONNS
static struct usrsock_conn_s g_usrsock_connections[CONFIG_NET_USRSOCK_CONNS];
#endif

/* A list of all free usrsock connections */

static dq_queue_t g_free_usrsock_connections;
static sem_t g_free_sem = SEM_INITIALIZER(1);

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
#ifdef CONFIG_NET_ALLOC_CONNS
  int i;
#endif

  /* The free list is protected by a semaphore (that behaves like a mutex). */

  _usrsock_semtake(&g_free_sem);
#ifdef CONFIG_NET_ALLOC_CONNS
  if (dq_peek(&g_free_usrsock_connections) == NULL)
    {
      conn = kmm_zalloc(sizeof(*conn) * CONFIG_NET_USRSOCK_CONNS);
      if (conn != NULL)
        {
          for (i = 0; i < CONFIG_NET_USRSOCK_CONNS; i++)
            {
              dq_addlast(&conn[i].sconn.node, &g_free_usrsock_connections);
            }
        }
    }
#endif

  conn = (FAR struct usrsock_conn_s *)
         dq_remfirst(&g_free_usrsock_connections);
  if (conn)
    {
      /* Make sure that the connection is marked as uninitialized */

      nxsem_init(&conn->resp.sem, 0, 1);
      conn->usockid = -1;
      conn->state = USRSOCK_CONN_STATE_UNINITIALIZED;

      /* Enqueue the connection into the active list */

      dq_addlast(&conn->sconn.node, &g_active_usrsock_connections);
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

  dq_rem(&conn->sconn.node, &g_active_usrsock_connections);

  /* Reset structure */

  nxsem_destroy(&conn->resp.sem);
  memset(conn, 0, sizeof(*conn));

  /* Free the connection */

  dq_addlast(&conn->sconn.node, &g_free_usrsock_connections);
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
      return (FAR struct usrsock_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: usrsock_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection for usrsock
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

  pstate->conn      = conn;
  pstate->result    = -EAGAIN;
  pstate->completed = false;
  pstate->unlock    = false;

  /* Set up the callback in the connection */

  pstate->cb = devif_callback_alloc(NULL, &conn->sconn.list,
                                    &conn->sconn.list_tail);
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

  devif_conn_callback_free(NULL, pstate->cb, &conn->sconn.list,
                           &conn->sconn.list_tail);
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
#ifndef CONFIG_NET_ALLOC_CONNS
  FAR struct usrsock_conn_s *conn;
  int i;

  for (i = 0; i < CONFIG_NET_USRSOCK_CONNS; i++)
    {
      conn = &g_usrsock_connections[i];

      /* Mark the connection closed and move it to the free list */

      conn->usockid = -1;
      conn->state   = USRSOCK_CONN_STATE_UNINITIALIZED;
      dq_addlast(&conn->sconn.node, &g_free_usrsock_connections);
    }
#endif

  /* Register /dev/usrsock character device. */

  usrsockdev_register();
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
