/****************************************************************************
 * net/tcp/tcp_close.c
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

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "tcp/tcp.h"
#include "socket/socket.h"
#include "utils/utils.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_close_work
 ****************************************************************************/

static void tcp_close_work(FAR void *param)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)param;

  /* The close event handler queues this work before tcp_appsend() acts on
   * its result: a TCP_ABORT sets TCP_CLOSED only after the handler
   * returns.  Look at the state under the same locks, or the work, run
   * on another CPU at once, would see the old state, free nothing, and
   * no one would free the connection later.
   */

  conn_dev_lock(&conn->sconn, conn->dev);
  conn->flags &= ~TCP_CLOSE_ARRANGED;
  if (conn->crefs == 0 && conn->tcpstateflags == TCP_CLOSED)
    {
      /* Stop the network monitor for all sockets */

      tcp_stop_monitor(conn, TCP_TXCLOSE);
      conn_dev_unlock(&conn->sconn, conn->dev);
      tcp_free(conn);
      return;
    }

  conn_dev_unlock(&conn->sconn, conn->dev);
}

/****************************************************************************
 * Name: tcp_close_abortive
 *
 * Description:
 *   Return true if the close is abortive: SO_LINGER is on with a time of
 *   zero.  POSIX and Linux: the connection is then reset (RST), the queued
 *   data is dropped, and no FIN is sent.
 *
 ****************************************************************************/

static bool tcp_close_abortive(FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_SOLINGER
  return _SO_GETOPT(conn->sconn.s_options, SO_LINGER) &&
         conn->sconn.s_linger == 0;
#else
  UNUSED(conn);
  return false;
#endif
}

/****************************************************************************
 * Name: tcp_close_eventhandler
 ****************************************************************************/

static uint32_t tcp_close_eventhandler(FAR struct net_driver_s *dev,
                                       FAR void *pvpriv, uint32_t flags)
{
  FAR struct tcp_conn_s *conn = pvpriv;

  ninfo("flags: %" PRIx32 "\n", flags);

  /* TCP_DISCONN_EVENTS:
   *   TCP_ABORT:    The remote host has aborted the connection
   *   TCP_TIMEDOUT: The remote did not respond, the connection timed out
   *   NETDEV_DOWN:  The network device went down
   *
   * TCP_TXCLOSE ends the close only when the peer acknowledged our FIN in
   * LAST_ACK (the state is TCP_CLOSED then).  The callback of shutdown()
   * runs before this one and sets TCP_TXCLOSE to send its FIN: this
   * callback must stay until the connection ends, or no one frees it.
   */

  if ((flags & TCP_DISCONN_EVENTS) != 0 ||
      ((flags & TCP_TXCLOSE) != 0 && conn->tcpstateflags == TCP_CLOSED))
    {
      /* The disconnection is complete.  Wake up the waiting thread with an
       * appropriate result.  Success is returned in these cases:
       *
       *   NOTE:  The underlying connection, however, will persist, waiting
       *   for the FIN to be returned by the remote in the TIME_WAIT state.
       *
       * * TCP_ABORT is less likely but still means that the socket was
       *   closed, albeit abnormally due to a RST from the remote.
       *
       * * TCP_TIMEDOUT would be reported in this context if there is no
       *   ACK response to the FIN in the FIN_WAIT_2 state.  The socket will
       *   again be closed abnormally.
       *
       * This is the only true error case.
       *
       * * NETDEV_DOWN would indicate that the network went down before the
       *   close completed.  A non-standard ENODEV error will be returned
       *   in this case.  The socket will be left in a limbo state if the
       *   network is taken down but should recover later when the
       *   NETWORK_DOWN event is processed further.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* No socket refers to the connection and its device sends
           * nothing more.  Close it now: tcp_close_work() frees only a
           * connection in TCP_CLOSED, and after this callback is freed no
           * one frees it.  A connection in TIME_WAIT, for example, would
           * become TCP_CLOSED later and stay in the active list.
           */

          conn->tcpstateflags = TCP_CLOSED;
        }

      goto end_wait;
    }

  /* Data that the application did not read, or that comes after the
   * close, ends the connection with a RST at once (RFC 1122, section
   * 4.2.2.13), also while our own data waits to be sent: the peer may
   * wait for us to read before it reads, and the two connections would
   * keep their buffers for ever.  An abortive close (SO_LINGER with a
   * time of zero) also sends a RST at once, not a FIN.
   */

  else if (conn->readahead != NULL || (flags & TCP_NEWDATA) != 0 ||
           tcp_close_abortive(conn))
    {
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      if (conn->sndcb != NULL)
        {
          conn->sndcb->flags = 0;
          conn->sndcb->event = NULL;
          conn->sndcb = NULL;
        }
#endif

      /* Set TCP_ABORT to send the RST, and free the rx buffers now */

      dev->d_len = 0;
      flags = (flags & ~TCP_NEWDATA) | TCP_ABORT;
      tcp_free_rx_buffers(conn);
      goto end_wait;
    }

  /* Check if all outstanding bytes have been ACKed.
   *
   * Note: in case of passive close, this ensures our FIN is acked.
   */

  else if (conn->tx_unacked != 0
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
           || !sq_empty(&conn->write_q)
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */
          )
    {
      /* No... we are still waiting for ACKs.  Drop any received data, but
       * do not yet report TCP_TXCLOSE in the response.
       */

      dev->d_len = 0;
      flags &= ~TCP_NEWDATA;
      ninfo("waiting for ack\n");
    }
  else
    {
      /* Note: the following state shouldn't reach here because
       *
       * CLOSING, LAST_ACK
       *   should have tx_unacked != 0, already handled above
       *
       * CLOSED, TIME_WAIT
       *   a TCP_TXCLOSE callback should have already cleared this callback
       *   when transitioning to these states.
       */

      DEBUGASSERT(conn->tcpstateflags == TCP_ESTABLISHED ||
                  conn->tcpstateflags == TCP_CLOSE_WAIT ||
                  conn->tcpstateflags == TCP_FIN_WAIT_1 ||
                  conn->tcpstateflags == TCP_FIN_WAIT_2);

      /* Make sure that TCP_TXCLOSE is set in the response */

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

      dev->d_len = 0;
      if ((conn->shutdown & SHUT_WR) == 0)
        {
          flags |= TCP_TXCLOSE;

          /* Avoid sending multiple FIN */

          conn->shutdown |= SHUT_WR;
        }
    }

  UNUSED(conn);           /* May not be used */
  return flags;

end_wait:
  if (conn->clscb != NULL)
    {
      tcp_callback_free(conn, conn->clscb);
      conn->clscb = NULL;
    }

  /* Free network resources */

  conn->flags |= TCP_CLOSE_ARRANGED;
  work_queue(LPWORK, &conn->clswork, tcp_close_work, conn, 0);

  return flags;
}

/****************************************************************************
 * Name: tcp_close_waits
 *
 * Description:
 *   Return true if a close of the connection must wait for its close
 *   callback (to send the queued data and the FIN, or to see the FIN
 *   acknowledged).
 *
 ****************************************************************************/

static bool tcp_close_waits(FAR struct tcp_conn_s *conn)
{
  return conn->tcpstateflags == TCP_ESTABLISHED ||
         conn->tcpstateflags == TCP_FIN_WAIT_1 ||
         conn->tcpstateflags == TCP_FIN_WAIT_2 ||
         conn->tcpstateflags == TCP_LAST_ACK ||
         conn->tcpstateflags == TCP_CLOSE_WAIT;
}

/****************************************************************************
 * Name: tcp_close_disconnect
 *
 * Description:
 *   Break any current TCP connection
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

static inline int tcp_close_disconnect(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn;
  bool reclaimed;
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  conn = psock->s_conn;

  conn_dev_lock(&conn->sconn, conn->dev);

  /* No socket refers to the connection any more: stop the monitor now, not
   * when the connection is freed after TIME_WAIT.  The close callback can
   * then use its callback structure.
   */

  tcp_stop_monitor(conn, TCP_TXCLOSE);

  /* TCP_ESTABLISHED
   *   We need to initiate an active close and wait for its completion.
   *
   * TCP_LAST_ACK
   *   We still need to wait for the ACK for our FIN, possibly
   *   retransmitting the FIN, before disposing the connection.
   */

  if (tcp_close_waits(conn))
    {
      conn->clscb = tcp_callback_alloc(conn);
      if (conn->clscb == NULL)
        {
          /* All callbacks are in use: free a connection that only waits
           * for the end of its close (for example in TIME_WAIT), or this
           * one goes without a FIN and its queued data is lost.
           * tcp_reclaim() takes a device lock and the list lock: not with
           * our locks held.  It does not take this connection: it has no
           * close callback.
           */

          conn_dev_unlock(&conn->sconn, conn->dev);
          reclaimed = tcp_reclaim(NULL);
          conn_dev_lock(&conn->sconn, conn->dev);

          if (reclaimed && tcp_close_waits(conn))
            {
              conn->clscb = tcp_callback_alloc(conn);
            }
        }
    }

  /* Discard our reference to the connection */

  conn->crefs = 0;

  if (conn->clscb != NULL)
    {
      /* Set up to receive TCP data event callbacks */

      conn->clscb->flags = TCP_NEWDATA | TCP_ACKDATA |
                           TCP_POLL | TCP_TXCLOSE | TCP_DISCONN_EVENTS;
      conn->clscb->event = tcp_close_eventhandler;
      conn->clscb->priv  = conn; /* reference for event handler to free cb */

#ifdef CONFIG_NET_SOLINGER
      /* SO_LINGER
       *   Lingers on a close() if data is present. This option controls the
       *   action taken when unsent messages queue on a socket and close() is
       *   performed. If SO_LINGER is set, the system shall block the calling
       *   thread during close() until it can transmit the data or until the
       *   time expires. If SO_LINGER is not specified, and close() is
       *   issued, the system handles the call in a way that allows the
       *   calling thread to continue as quickly as possible. This option
       *   takes a linger structure, as defined in the <sys/socket.h> header,
       *   to specify the state of the option and linger interval.
       */

      if (_SO_GETOPT(conn->sconn.s_options, SO_LINGER))
        {
          conn->ltimeout = clock_systime_ticks() +
                           DSEC2TICK(conn->sconn.s_linger);

          /* Update RTO timeout if the work exceeds expire */

          if (conn->sconn.s_linger == 0)
            {
              /* Directly set timeout to avoid the priority of the
               * schedule tcp_timer_expiry and tcp_send_txnotify
               */

              conn->timeout = true;
            }
          else
            {
              tcp_update_timer(conn);
            }
        }
#endif

      /* Notify the device driver of the availability of TX data */

      tcp_send_txnotify(psock, conn);
      conn_dev_unlock(&conn->sconn, conn->dev);
    }
  else if (conn->tcpstateflags == TCP_SYN_SENT ||
           conn->tcpstateflags == TCP_SYN_RCVD)
    {
      /* Free network resources */

      conn->flags |= TCP_CLOSE_ARRANGED;
      conn->tcpstateflags = TCP_CLOSED;
      work_queue(LPWORK, &conn->clswork, tcp_close_work, conn,
                 HSEC2TICK(conn->timer));
      conn_dev_unlock(&conn->sconn, conn->dev);
    }
  else
    {
      conn_dev_unlock(&conn->sconn, conn->dev);

      /* Free network resources */

      tcp_free(conn);
    }

  psock->s_conn = NULL;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_close
 *
 * Description:
 *   Break any current TCP connection
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int tcp_close(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;

  /* Perform the disconnection now */

  tcp_unlisten(conn); /* No longer accepting connections */

  /* Break any current connections and close the socket */

  return tcp_close_disconnect(psock);
}

#endif /* CONFIG_NET_TCP */
