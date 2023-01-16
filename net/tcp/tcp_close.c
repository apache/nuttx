/****************************************************************************
 * net/tcp/tcp_close.c
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_close_work
 ****************************************************************************/

static void tcp_close_work(FAR void *param)
{
  FAR struct tcp_conn_s *conn = (FAR struct tcp_conn_s *)param;

  net_lock();

  if (conn && conn->crefs == 0)
    {
      /* Stop the network monitor for all sockets */

      tcp_stop_monitor(conn, TCP_CLOSE);
      tcp_free(conn);
    }

  net_unlock();
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
   *   TCP_CLOSE:    The remote host has closed the connection
   *   TCP_ABORT:    The remote host has aborted the connection
   *   TCP_TIMEDOUT: The remote did not respond, the connection timed out
   *   NETDEV_DOWN:  The network device went down
   */

  if ((flags & TCP_DISCONN_EVENTS) != 0)
    {
      /* The disconnection is complete.  Wake up the waiting thread with an
       * appropriate result.  Success is returned in these cases:
       *
       * * TCP_CLOSE indicates normal successful closure.  The TCP_CLOSE
       *   event is sent when the remote ACKs the outgoing FIN in the
       *   FIN_WAIT_1 state.  That is the appropriate time for the
       *   application to close the socket.
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
       * do not yet report TCP_CLOSE in the response.
       */

      dev->d_len = 0;
      flags &= ~TCP_NEWDATA;
      ninfo("waiting for ack\n");
    }
  else
    {
      /* Note: the following state shouldn't reach here because
       *
       * FIN_WAIT_1, CLOSING, LAST_ACK
       *   should have tx_unacked != 0, already handled above
       *
       * CLOSED, TIME_WAIT
       *   a TCP_CLOSE callback should have already cleared this callback
       *   when transitioning to these states.
       *
       * FIN_WAIT_2
       *   new data is dropped by tcp_input without invoking tcp_callback.
       *   timer is handled by tcp_timer without invoking tcp_callback.
       *   TCP_CLOSE is handled above.
       */

      DEBUGASSERT(conn->tcpstateflags == TCP_ESTABLISHED);

      /* Drop data received in this state and make sure that TCP_CLOSE
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
      flags = (flags & ~TCP_NEWDATA) | TCP_CLOSE;
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

  work_queue(LPWORK, &conn->clswork, tcp_close_work, conn, 0);

  return flags;
}

/****************************************************************************
 * Name: tcp_close_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we have data ready to
 *   be sent (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *   conn  - The TCP connection structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void tcp_close_txnotify(FAR struct socket *psock,
                                      FAR struct tcp_conn_s *conn)
{
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  /* If both IPv4 and IPv6 support are enabled, then we will need to select
   * the device driver using the appropriate IP domain.
   */

  if (psock->s_domain == PF_INET)
#endif
    {
      /* Notify the device driver that send data is available */

      netdev_ipv4_txnotify(conn->u.ipv4.laddr, conn->u.ipv4.raddr);
    }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  else /* if (psock->s_domain == PF_INET6) */
#endif /* CONFIG_NET_IPv4 */
    {
      /* Notify the device driver that send data is available */

      DEBUGASSERT(psock->s_domain == PF_INET6);
      netdev_ipv6_txnotify(conn->u.ipv6.laddr, conn->u.ipv6.raddr);
    }
#endif /* CONFIG_NET_IPv6 */
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

  net_lock();

  conn = (FAR struct tcp_conn_s *)psock->s_conn;
  DEBUGASSERT(conn != NULL);

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
       conn->tcpstateflags == TCP_LAST_ACK) &&
      (conn->clscb = tcp_callback_alloc(conn)) != NULL)
    {
      /* Set up to receive TCP data event callbacks */

      conn->clscb->flags = TCP_NEWDATA | TCP_ACKDATA |
                           TCP_POLL | TCP_DISCONN_EVENTS;
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

      tcp_close_txnotify(psock, conn);
    }
  else
    {
      /* Stop the network monitor for all sockets */

      tcp_stop_monitor(conn, TCP_CLOSE);

      /* Free network resources */

      tcp_free(conn);
    }

  psock->s_conn = NULL;

  net_unlock();
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
