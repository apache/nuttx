/****************************************************************************
 * net/socket/net_monitor.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "tcp/tcp.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void connection_closed(FAR struct socket *psock, uint16_t flags);
static uint16_t connection_event(FAR struct net_driver_s *dev,
                                 FAR void *pvconn, FAR void *pvpriv,
                                 uint16_t flags);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: connection_closed
 *
 * Description:
 *   Called when a loss-of-connection event has occurred.
 *
 * Parameters:
 *   psock    The TCP socket structure associated.
 *   flags    Set of connection events events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock.
 *
 ****************************************************************************/

static void connection_closed(FAR struct socket *psock, uint16_t flags)
{
  /* These loss-of-connection events may be reported:
   *
   *   TCP_CLOSE: The remote host has closed the connection
   *   TCP_ABORT: The remote host has aborted the connection
   *   TCP_TIMEDOUT: Connection aborted due to too many retransmissions.
   *   NETDEV_DOWN: The network device went down
   *
   * And we need to set these two socket status bits appropriately:
   *
   *  _SF_CONNECTED==1 && _SF_CLOSED==0 - the socket is connected
   *  _SF_CONNECTED==0 && _SF_CLOSED==1 - the socket was gracefully disconnected
   *  _SF_CONNECTED==0 && _SF_CLOSED==0 - the socket was rudely disconnected
   */

  if ((flags & TCP_CLOSE) != 0)
    {
      /* The peer gracefully closed the connection.  Marking the
       * connection as disconnected will suppress some subsequent
       * ENOTCONN errors from receive.  A graceful disconnection is
       * not handle as an error but as an "end-of-file"
       */

      psock->s_flags &= ~_SF_CONNECTED;
      psock->s_flags |= _SF_CLOSED;
    }
  else if ((flags & (TCP_ABORT | TCP_TIMEDOUT | NETDEV_DOWN)) != 0)
    {
      /* The loss of connection was less than graceful.  This will (eventually)
       * be reported as an ENOTCONN error.
       */

      psock->s_flags &= ~(_SF_CONNECTED |_SF_CLOSED);
    }
}

/****************************************************************************
 * Name: connection_event
 *
 * Description:
 *   Some connection related event has occurred
 *
 * Parameters:
 *   dev      The device which as active when the event was detected.
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t connection_event(FAR struct net_driver_s *dev,
                                 FAR void *pvconn, FAR void *pvpriv,
                                 uint16_t flags)
{
  FAR struct socket *psock = (FAR struct socket *)pvpriv;

  if (psock)
    {
      nllvdbg("flags: %04x s_flags: %02x\n", flags, psock->s_flags);

      /* TCP_DISCONN_EVENTS: TCP_CLOSE, TCP_ABORT, TCP_TIMEDOUT, or
       * NETDEV_DOWN.  All loss-of-connection events.
       */

      if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          connection_closed(psock, flags);
        }

      /* TCP_CONNECTED: The socket is successfully connected */

      else if ((flags & TCP_CONNECTED) != 0)
        {
          /* Indicate that the socket is now connected */

          psock->s_flags |= _SF_CONNECTED;
          psock->s_flags &= ~_SF_CLOSED;
        }
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: net_startmonitor
 *
 * Description:
 *   Set up to receive TCP connection state changes for a given socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *
 * Returned Value:
 *   On success, net_startmonitor returns OK; On any failure,
 *   net_startmonitor will return a negated errno value.  The only failure
 *   that can occur is if the socket has already been closed and, in this
 *   case, -ENOTCONN is returned.
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

int net_startmonitor(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn;
  FAR struct devif_callback_s *cb;
  net_lock_t save;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* Check if the connection has already been closed before any callbacks
   * have been registered. (Maybe the connection is lost before accept has
   * registered the monitoring callback.)
   */

  save = net_lock();
  if (!(conn->tcpstateflags == TCP_ESTABLISHED ||
        conn->tcpstateflags == TCP_SYN_RCVD))
    {
      /* Invoke the TCP_CLOSE connection event now */

      (void)connection_event(NULL, conn, psock, TCP_CLOSE);

      /* Make sure that the monitor is stopped */

      conn->connection_private = NULL;
      conn->connection_devcb   = NULL;
      conn->connection_event   = NULL;

      /* And return -ENOTCONN to indicate the the monitor was not started
       * because the socket was already disconnected.
       */

      net_unlock(save);
      return -ENOTCONN;
    }

  DEBUGASSERT(conn->connection_event == NULL &&
              conn->connection_devcb == NULL);

  /* Allocate a callback structure that we will use to get callbacks if
   * the network goes down.
   */

  cb = tcp_monitor_callback_alloc(conn);
  if (cb != NULL)
    {
      cb->event = connection_event;
      cb->priv  = (void*)psock;
      cb->flags = NETDEV_DOWN;
    }

  conn->connection_devcb = cb;

  /* Set up to receive callbacks on connection-related events */

  conn->connection_private = (void*)psock;
  conn->connection_event   = connection_event;

  net_unlock(save);
  return OK;
}

/****************************************************************************
 * Name: net_stopmonitor
 *
 * Description:
 *   Stop monitoring TCP connection changes for a given socket
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

void net_stopmonitor(FAR struct tcp_conn_s *conn)
{
  net_lock_t save;

  DEBUGASSERT(conn);

  /* Free any allocated device event callback structure */

  save = net_lock();
  if (conn->connection_devcb)
    {
      tcp_monitor_callback_free(conn, conn->connection_devcb);
    }

  /* Nullify all connection event data */

  conn->connection_private = NULL;
  conn->connection_devcb   = NULL;
  conn->connection_event   = NULL;
  net_unlock(save);
}

/****************************************************************************
 * Name: net_lostconnection
 *
 * Description:
 *   Called when a loss-of-connection event has occurred.
 *
 * Parameters:
 *   psock    The TCP socket structure associated.
 *   flags    Set of connection events events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller holds the network lock (if not, it will be locked momentarily
 *   by this function).
 *
 ****************************************************************************/

void net_lostconnection(FAR struct socket *psock, uint16_t flags)
{
  net_lock_t save;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  /* Close the connection */

  save = net_lock();
  connection_closed(psock, flags);

  /* Stop the network monitor */

  net_stopmonitor((FAR struct tcp_conn_s *)psock->s_conn);
  net_unlock(save);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
