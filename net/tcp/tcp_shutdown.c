/****************************************************************************
 * net/tcp/tcp_shutdown.c
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
#ifdef CONFIG_NET_TCP

#include <errno.h>
#include <nuttx/debug.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "utils/utils.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_shutdown_eventhandler
 ****************************************************************************/

static uint32_t tcp_shutdown_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvpriv, uint32_t flags)
{
  FAR struct tcp_conn_s *conn = pvpriv;

  ninfo("flags: %" PRIx32 "\n", flags);

  /* The connection is gone: nothing more to send */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      tcp_callback_free(conn, conn->shdcb);
      conn->shdcb = NULL;
      return flags;
    }

  /* The FIN goes after the data that the application wrote before
   * shutdown(): wait while data waits in the write queue or is not yet
   * acknowledged, as close() does.  The send callback still sends it.
   * An ACK can come with new data: leave it to the reader (no d_len = 0).
   */

  if ((conn->shutdown & SHUT_WR) != 0 &&
      (conn->tx_unacked != 0
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
       || !sq_empty(&conn->write_q)
#endif
      ))
    {
      return flags;
    }

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* We don't need the send callback anymore.  Free it now: the
   * connection can stay in TIME_WAIT for a long time.
   */

  if (conn->sndcb != NULL)
    {
      tcp_callback_free(conn, conn->sndcb);
      conn->sndcb = NULL;
    }
#endif

  if ((conn->shutdown & SHUT_WR) != 0)
    {
      flags |= TCP_TXCLOSE;
    }

  if ((conn->shutdown & SHUT_RD) != 0)
    {
      flags |= TCP_RXCLOSE;
    }

  if (conn->shdcb != NULL)
    {
      tcp_callback_free(conn, conn->shdcb);
      conn->shdcb = NULL;
    }

  return flags;
}

/****************************************************************************
 * Name: tcp_send_fin
 *
 * Description:
 *   Send a FIN for TCP connection
 *
 * Input Parameters:
 *   conn - TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

static inline int tcp_send_fin(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn;
  bool reclaimed;
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  conn_dev_lock(&conn->sconn, conn->dev);
  if ((conn->tcpstateflags == TCP_ESTABLISHED ||
       conn->tcpstateflags == TCP_SYN_SENT ||
       conn->tcpstateflags == TCP_SYN_RCVD ||
       conn->tcpstateflags == TCP_CLOSE_WAIT))
    {
      /* A callback of an earlier shutdown() may still wait for the queued
       * data: it reads conn->shutdown when it runs, so use it again.
       */

      if (conn->shdcb == NULL &&
          (conn->shdcb = tcp_callback_alloc(conn)) == NULL)
        {
          /* All callbacks are in use: free a connection that only waits
           * for the end of its close (for example in TIME_WAIT), as close()
           * does.  tcp_reclaim() takes a device lock and the list lock: not
           * with our locks held.  It does not take this connection: a
           * socket refers to it.
           */

          conn_dev_unlock(&conn->sconn, conn->dev);
          reclaimed = tcp_reclaim(NULL);
          conn_dev_lock(&conn->sconn, conn->dev);

          if (!reclaimed ||
              (conn->tcpstateflags != TCP_ESTABLISHED &&
               conn->tcpstateflags != TCP_SYN_SENT &&
               conn->tcpstateflags != TCP_SYN_RCVD &&
               conn->tcpstateflags != TCP_CLOSE_WAIT) ||
              (conn->shdcb == NULL &&
               (conn->shdcb = tcp_callback_alloc(conn)) == NULL))
            {
              ret = -EBUSY;
              goto out;
            }
        }

      /* Set up to receive TCP data event callbacks */

      conn->shdcb->flags = TCP_POLL | TCP_ACKDATA | TCP_DISCONN_EVENTS;
      conn->shdcb->event = tcp_shutdown_eventhandler;
      conn->shdcb->priv  = conn; /* reference for event handler to free cb */

      /* Notify the device driver of the availability of TX data */

      tcp_send_txnotify(psock, conn);
    }

out:
  conn_dev_unlock(&conn->sconn, conn->dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_shutdown
 *
 * Description:
 *   Gracefully shutdown a TCP connection by sending a FIN
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   how   - Specifies the type of shutdown.
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int tcp_shutdown(FAR struct socket *psock, int how)
{
  FAR struct tcp_conn_s *conn;
  uint8_t oldshutdown;
  int ret;

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  if (!(how & SHUT_RDWR))
    {
      return -EOPNOTSUPP;
    }

  oldshutdown     = conn->shutdown;
  conn->shutdown |= how;

  tcp_unlisten(psock->s_conn); /* No longer accepting connections */

  ret = tcp_send_fin(psock);
  if (ret < 0)
    {
      /* No FIN will be sent: do not leave the connection marked as shut
       * down, or close() sends no FIN either ("avoid multiple FIN") and
       * the connection stays for ever once the peer closes (CLOSE_WAIT).
       */

      conn->shutdown = oldshutdown;
    }

  return ret;
}

#endif /* CONFIG_NET_TCP */
