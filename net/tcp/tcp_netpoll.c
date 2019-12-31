/****************************************************************************
 * net/tcp/tcp_netpoll.c
 *
 *   Copyright (C) 2008-2009, 2011-2016, 2018 Gregory Nutt. All rights
 *     reserved.
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
#include <assert.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/net/net.h>
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

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(info == NULL || (info->psock != NULL && info->fds != NULL));

  /* 'priv' might be null in some race conditions (?) */

  if (info != NULL)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & (TCP_NEWDATA | TCP_BACKLOG)) != 0)
        {
          eventset |= POLLIN & info->fds->events;
        }

      /* Check for a loss of connection events. */

      if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          /* Mark that the connection has been lost */

          tcp_lost_connection(info->psock, info->cb, flags);
          eventset |= (POLLERR | POLLHUP);
        }

      /* A poll is a sign that we are free to send data. */

      else if (psock_tcp_cansend(info->psock) >= 0)
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

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->psock != NULL)
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

  info->psock  = psock;
  info->fds    = fds;
  info->cb     = cb;

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags    = TCP_DISCONN_EVENTS;
  cb->priv     = (FAR void *)info;
  cb->event    = tcp_poll_eventhandler;

  if ((fds->events & POLLOUT) != 0)
    {
      cb->flags |= TCP_POLL;
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

  if (!IOB_QEMPTY(&conn->readahead) || tcp_backlogavailable(conn))
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

  if (!_SS_ISCONNECTED(psock->s_flags) && !_SS_ISLISTENING(psock->s_flags))
    {
      /* We were previously connected but lost the connection either due
       * to a graceful shutdown by the remote peer or because of some
       * exceptional event.
       */

      fds->revents |= (POLLERR | POLLHUP);
    }
  else if (_SS_ISCONNECTED(psock->s_flags) && psock_tcp_cansend(psock) >= 0)
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

      info->psock = NULL;
    }

  return OK;
}
