/****************************************************************************
 * net/local/local_accept.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_waitlisten
 ****************************************************************************/

static int local_waitlisten(FAR struct local_conn_s *server)
{
  int ret;

  /* Loop until a connection is requested or we receive a signal */

  while (dq_empty(&server->u.server.lc_waiters))
    {
      /* No.. wait for a connection or a signal */

      ret = net_sem_wait(&server->lc_waitsem);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* There is an accept conn waiting to be processed */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_accept
 *
 * Description:
 *   This function implements accept() for Unix domain sockets.  See the
 *   description of accept() for further information.
 *
 * Input Parameters:
 *   psock    The listening Unix domain socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   newconn  The new, accepted  Unix domain connection structure
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *   See the description of accept of the possible errno values in the
 *   description of accept().
 *
 * Assumptions:
 *   Network is NOT locked.
 *
 ****************************************************************************/

int local_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                 FAR socklen_t *addrlen, FAR struct socket *newsock,
                 int flags)
{
  FAR struct local_conn_s *server = psock->s_conn;
  FAR struct local_conn_s *conn;
  FAR dq_entry_t *waiter;
  bool nonblock = !!(flags & SOCK_NONBLOCK);
  int ret = OK;

  /* Some sanity checks */

  DEBUGASSERT(newsock && !newsock->s_conn);

  /* Is the socket a stream? */

  if (psock->s_domain != PF_LOCAL || psock->s_type != SOCK_STREAM)
    {
      return -EOPNOTSUPP;
    }

  if (server->lc_proto != SOCK_STREAM ||
      server->lc_state != LOCAL_STATE_LISTENING)
    {
      return -EOPNOTSUPP;
    }

  /* Loop as necessary if we have to wait for a connection */

  for (; ; )
    {
      /* Are there pending connections.  Remove the accpet from the
       * head of the waiting list.
       */

      waiter = dq_remfirst(&server->u.server.lc_waiters);
      if (waiter)
        {
          conn = container_of(waiter, struct local_conn_s,
                              u.accept.lc_waiter);

          /* Decrement the number of pending accpets */

          DEBUGASSERT(server->u.server.lc_pending > 0);
          server->u.server.lc_pending--;

          /* Setup the accpet socket structure */

          newsock->s_domain = psock->s_domain;
          newsock->s_type   = SOCK_STREAM;
          newsock->s_sockif = psock->s_sockif;
          newsock->s_conn   = (FAR void *)conn;

          /* Return the address family */

          if (addr != NULL && conn->lc_peer != NULL)
            {
              ret = local_getaddr(conn->lc_peer, addr, addrlen);
            }

          if (ret == OK && nonblock)
            {
              ret = local_set_nonblocking(conn);
            }

          return ret;
        }

      /* No.. then there should be no pending connections */

      DEBUGASSERT(server->u.server.lc_pending == 0);

      /* Was the socket opened non-blocking? */

      if (_SS_ISNONBLOCK(server->lc_conn.s_flags))
        {
          /* Yes.. return EAGAIN */

          return -EAGAIN;
        }

      /* Otherwise, listen for a connection and try again. */

      ret = local_waitlisten(server);
      if (ret < 0)
        {
          return ret;
        }
    }
}
