/****************************************************************************
 * net/tcp/tcp_netpoll.c
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
#include <assert.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>
#include <nuttx/semaphore.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "inet/inet.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual TCP receive operation via
 *   the device interface layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t tcp_poll_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *conn,
                                      FAR void *pvpriv, uint16_t flags)
{
  FAR struct tcp_poll_s *info = (FAR struct tcp_poll_s *)pvpriv;
  int reason;

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(info == NULL || (info->conn != NULL && info->fds != NULL));

  /* 'priv' might be null in some race conditions (?) */

  if (info != NULL)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & (TCP_NEWDATA | TCP_BACKLOG)) != 0)
        {
          eventset |= POLLIN & info->fds->events;
        }

      /* Non-blocking connection */

      if ((flags & TCP_CONNECTED) != 0)
        {
          eventset |= POLLOUT & info->fds->events;
        }

      /* Check for a loss of connection events. */

      if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          /* TCP_TIMEDOUT: Connection aborted due to too many
           *               retransmissions.
           */

          if ((flags & TCP_TIMEDOUT) != 0)
            {
              /* Indicate that the connection timedout?) */

              reason = ETIMEDOUT;
            }

          else if ((flags & NETDEV_DOWN) != 0)
            {
              /* The network device went down.  Indicate that the remote host
               * is unreachable.
               */

              reason = ENETUNREACH;
            }

          /* TCP_CLOSE: The remote host has closed the connection
           * TCP_ABORT: The remote host has aborted the connection
           */

          else
            {
              /* Indicate that remote host refused the connection */

              reason = ECONNREFUSED;
            }

#ifdef CONFIG_NET_SOCKOPTS
          info->conn->sconn.s_error = reason;
#endif
          set_errno(reason);

          /* Mark that the connection has been lost */

          tcp_lost_connection(info->conn, info->cb, flags);
          eventset |= (POLLERR | POLLHUP);
        }

      /* A poll is a sign that we are free to send data. */

      /* Wake up poll() speculatively on TCP_ACKDATA.
       * Note: our event handler is usually executed before
       * psock_send_eventhandler, which might free IOBs/WRBs on TCP_ACKDATA.
       * Revisit: consider some kind of priority for devif callback to allow
       * this callback to be inserted after psock_send_eventhandler.
       */

      else if (psock_tcp_cansend(info->conn) >= 0
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS)
               || (flags & TCP_ACKDATA) != 0
#endif
              )
        {
          eventset |= (POLLOUT & info->fds->events);
        }

      /* Awaken the caller of poll() if requested event occurred. */

      if (eventset != 0)
        {
          /* Stop further callbacks */

          info->cb->flags   = 0;
          info->cb->priv    = NULL;
          info->cb->event   = NULL;

          info->fds->revents |= eventset;
          nxsem_post(info->fds->sem);
        }
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one TCP/IP socket
 *
 * Input Parameters:
 *   psock - The TCP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int tcp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  FAR struct tcp_poll_s *info;
  FAR struct devif_callback_s *cb;
  bool nonblock_conn;
  int ret = OK;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn || !fds)
    {
      return -EINVAL;
    }
#endif

  /* Some of the  following must be atomic */

  net_lock();

  /* Non-blocking connection ? */

  nonblock_conn = (conn->tcpstateflags == TCP_SYN_SENT &&
                   _SS_ISNONBLOCK(conn->sconn.s_flags));

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->conn != NULL)
    {
      if (++info >= &conn->pollinfo[CONFIG_NET_TCP_NPOLLWAITERS])
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }
    }

  /* Allocate a TCP/IP callback structure */

  cb = tcp_callback_alloc(conn);
  if (!cb)
    {
      ret = -EBUSY;
      goto errout_with_lock;
    }

  /* Initialize the poll info container */

  info->conn = conn;
  info->fds  = fds;
  info->cb   = cb;

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags    = TCP_DISCONN_EVENTS;
  cb->priv     = (FAR void *)info;
  cb->event    = tcp_poll_eventhandler;

  if ((fds->events & POLLOUT) != 0)
    {
      cb->flags |= TCP_POLL
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS)
                   | TCP_ACKDATA
#endif
                   ;

      /* Monitor the connected event */

      if (nonblock_conn)
        {
          cb->flags |= TCP_CONNECTED;
        }
    }

  if ((fds->events & POLLIN) != 0)
    {
      cb->flags |= TCP_NEWDATA | TCP_BACKLOG;
    }

  /* Save the reference in the poll info structure as fds private as well
   * for use during poll teardown as well.
   */

  fds->priv    = (FAR void *)info;

  /* Check for read data or backlogged connection availability now */

  if (conn->readahead != NULL || tcp_backlogavailable(conn))
    {
      /* Normal data may be read without blocking. */

      fds->revents |= (POLLRDNORM & fds->events);
    }

  /* Check for a loss of connection events.  We need to be careful here.
   * There are four possibilities:
   *
   * 1) The socket is connected and we are waiting for data availability
   *    events.
   *
   *    __SS_ISCONNECTED(f) == true
   *    __SS_ISLISTENING(f) == false
   *    __SS_ISCLOSED(f)    == false
   *
   *    Action: Wait for data availability events
   *
   * 2) This is a listener socket that was never connected and we are
   *    waiting for connection events.
   *
   *    __SS_ISCONNECTED(f) == false
   *    __SS_ISLISTENING(f) == true
   *    __SS_ISCLOSED(f)    == false
   *
   *    Action: Wait for connection events
   *
   * 3) This socket was previously connected, but the peer has gracefully
   *    closed the connection.
   *
   *    __SS_ISCONNECTED(f) == false
   *    __SS_ISLISTENING(f) == false
   *    __SS_ISCLOSED(f)    == true
   *
   *    Action: Return with POLLHUP|POLLERR events
   *
   * 4) This socket was previously connected, but we lost the connection
   *    due to some exceptional event.
   *
   *    __SS_ISCONNECTED(f) == false
   *    __SS_ISLISTENING(f) == false
   *    __SS_ISCLOSED(f)    == false
   *
   *    Action: Return with POLLHUP|POLLERR events
   */

  if (!nonblock_conn && !_SS_ISCONNECTED(conn->sconn.s_flags) &&
      !_SS_ISLISTENING(conn->sconn.s_flags))
    {
      /* We were previously connected but lost the connection either due
       * to a graceful shutdown by the remote peer or because of some
       * exceptional event.
       */

      fds->revents |= (POLLERR | POLLHUP);
    }
  else if (_SS_ISCONNECTED(conn->sconn.s_flags) &&
           psock_tcp_cansend(conn) >= 0)
    {
      fds->revents |= (POLLWRNORM & fds->events);
    }

  /* Check if any requested events are already in effect */

  if (fds->revents != 0)
    {
      /* Yes.. then signal the poll logic */

      nxsem_post(fds->sem);
    }

errout_with_lock:
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: tcp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an TCP/IP socket
 *
 * Input Parameters:
 *   psock - The TCP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int tcp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  FAR struct tcp_poll_s *info;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct tcp_poll_s *)fds->priv;
  DEBUGASSERT(info != NULL && info->fds != NULL && info->cb != NULL);
  if (info != NULL)
    {
      /* Release the callback */

      tcp_callback_free(conn, info->cb);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      info->conn = NULL;
    }

  return OK;
}
