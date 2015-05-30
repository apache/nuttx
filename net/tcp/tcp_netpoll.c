/****************************************************************************
 * net/tcp/tcp_netpoll.c
 *
 *   Copyright (C) 2008-2009, 2011-2015 Gregory Nutt. All rights reserved.
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
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

#include <devif/devif.h>
#include <socket/socket.h>
#include "tcp/tcp.h"

#ifdef HAVE_TCP_POLL

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This is an allocated container that holds the poll-related information */

struct tcp_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  struct pollfd *fds;              /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_poll_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   TCP receive operation via by the device interface layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static uint16_t tcp_poll_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                                   FAR void *pvpriv, uint16_t flags)
{
  FAR struct tcp_poll_s *info = (FAR struct tcp_poll_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  DEBUGASSERT(!info || (info->psock && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & (TCP_NEWDATA | TCP_BACKLOG)) != 0)
        {
          eventset |= POLLIN & info->fds->events;
        }

      /* A poll is a sign that we are free to send data. */

      if ((flags & TCP_POLL) != 0)
        {
          eventset |= (POLLOUT & info->fds->events);
        }

      /* Check for a loss of connection events. */

      if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          /* Mark that the connection has been lost */

          net_lostconnection(info->psock, flags);
          eventset |= (POLLERR | POLLHUP);
        }

      /* Awaken the caller of poll() is requested event occurred. */

      if (eventset)
        {
          info->fds->revents |= eventset;
          sem_post(info->fds->sem);
        }
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tcp_pollsetup
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
  net_lock_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a container to hold the poll information */

  info = (FAR struct tcp_poll_s *)kmm_malloc(sizeof(struct tcp_poll_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Some of the  following must be atomic */

  flags = net_lock();

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

  cb->flags    = (TCP_NEWDATA | TCP_BACKLOG | TCP_POLL | TCP_DISCONN_EVENTS);
  cb->priv     = (FAR void *)info;
  cb->event    = tcp_poll_interrupt;

  /* Save the reference in the poll info structure as fds private as well
   * for use during poll teardown as well.
   */

  fds->priv    = (FAR void *)info;

#ifdef CONFIG_NET_TCPBACKLOG
  /* Check for read data or backlogged connection availability now */

  if (!IOB_QEMPTY(&conn->readahead) || tcp_backlogavailable(conn))
#else
  /* Check for read data availability now */

  if (!IOB_QEMPTY(&conn->readahead))
#endif
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

  /* Check if any requested events are already in effect */

  if (fds->revents != 0)
    {
      /* Yes.. then signal the poll logic */

      sem_post(fds->sem);
    }

  net_unlock(flags);
  return OK;

errout_with_lock:
  kmm_free(info);
  net_unlock(flags);
  return ret;
}

/****************************************************************************
 * Function: tcp_pollteardown
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
  net_lock_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct tcp_poll_s *)fds->priv;
  DEBUGASSERT(info && info->fds && info->cb);
  if (info)
    {
      /* Release the callback */

      flags = net_lock();
      tcp_callback_free(conn, info->cb);
      net_unlock(flags);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

#endif /* HAVE_TCP_POLL */
