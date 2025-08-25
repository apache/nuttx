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
#include <debug.h>
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

  conn->flags &= ~TCP_CLOSE_ARRANGED;
  if (conn->crefs == 0 && conn->tcpstateflags == TCP_CLOSED)
    {
      /* Stop the network monitor for all sockets */

      conn_dev_lock(&conn->sconn, conn->dev);
      tcp_stop_monitor(conn, TCP_TXCLOSE);
      conn_dev_unlock(&conn->sconn, conn->dev);
      tcp_free(conn);
    }
}

/****************************************************************************
 * Name: tcp_close_eventhandler
 ****************************************************************************/

static uint16_t tcp_close_eventhandler(FAR struct net_driver_s *dev,
                                       FAR void *pvpriv, uint16_t flags)
{
  FAR struct tcp_conn_s *conn = pvpriv;

  ninfo("flags: %04x\n", flags);

  /* TCP_DISCONN_EVENTS:
   *   TCP_ABORT:    The remote host has aborted the connection
   *   TCP_TIMEDOUT: The remote did not respond, the connection timed out
   *   NETDEV_DOWN:  The network device went down
   */

  if ((flags & (TCP_TXCLOSE | TCP_DISCONN_EVENTS)) != 0)
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

      /* Drop data received in this state and make sure that TCP_TXCLOSE
       * is set in the response
       */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
      /* We don't need the send callback anymore. */

      if (conn->sndcb != NULL)
        {
          conn->sndcb->flags = 0;
          conn->sndcb->event = NULL;

          /* The callback will be freed by tcp_free. */

          conn->sndcb = NULL;
        }
#endif

      dev->d_len = 0;
      if (conn->readahead != NULL || (flags & TCP_NEWDATA) != 0)
        {
          /* We need to send RST when read-ahead buffer data is not consumed
           * or new data is coming.
           * Set TCP_ABORT flag to trigger sending RST.
           */

          flags = flags & ~TCP_NEWDATA;
          flags |= TCP_ABORT;

          /* Free rx buffers of the connection immediately */

          tcp_free_rx_buffers(conn);

          goto end_wait;
        }
      else if ((conn->shutdown & SHUT_WR) == 0)
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
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  conn = psock->s_conn;

  conn_dev_lock(&conn->sconn, conn->dev);

  /* Discard our reference to the connection */

  conn->crefs = 0;

  /* TCP_ESTABLISHED
   *   We need to initiate an active close and wait for its completion.
   *
   * TCP_LAST_ACK
   *   We still need to wait for the ACK for our FIN, possibly
   *   retransmitting the FIN, before disposing the connection.
   */

  if ((conn->tcpstateflags == TCP_ESTABLISHED ||
       conn->tcpstateflags == TCP_FIN_WAIT_1 ||
       conn->tcpstateflags == TCP_FIN_WAIT_2 ||
       conn->tcpstateflags == TCP_LAST_ACK ||
       conn->tcpstateflags == TCP_CLOSE_WAIT) &&
      (conn->clscb = tcp_callback_alloc(conn)) != NULL)
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

          tcp_update_timer(conn);
        }
#endif

      /* Notify the device driver of the availability of TX data */

      tcp_send_txnotify(psock, conn);
      conn_dev_unlock(&conn->sconn, conn->dev);
    }
  else
    {
      /* Stop the network monitor for all sockets */

      tcp_stop_monitor(conn, TCP_TXCLOSE);
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
