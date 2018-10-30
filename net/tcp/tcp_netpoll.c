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
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "inet/inet.h"
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
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  int16_t key;                     /* Needed to cancel pending notification */
#endif
};

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

      /* A poll is a sign that we are free to send data.
       * REVISIT: This is bogus:  If CONFIG_TCP_WRITE_BUFFERS=y then
       * we never have to wait to send; otherwise, we always have to
       * wait to send.  Receiving a poll is irrelevant.
       */

      if ((flags & TCP_POLL) != 0)
        {
          eventset |= (POLLOUT & info->fds->events);
        }

      /* Check for a loss of connection events. */

      if ((flags & TCP_DISCONN_EVENTS) != 0)
        {
          /* Mark that the connection has been lost */

          tcp_lost_connection(info->psock, info->cb, flags);
          eventset |= (POLLERR | POLLHUP);
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
 * Name: tcp_iob_work
 *
 * Description:
 *   Work thread callback function execute when an IOB because available.
 *
 * Input Parameters:
 *   psock - Socket state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
static inline void tcp_iob_work(FAR void *arg)
{
  FAR struct work_notifier_entry_s *entry;
  FAR struct work_notifier_s *ninfo;
  FAR struct tcp_poll_s *pinfo;
  FAR struct socket *psock;
  FAR struct pollfd *fds;

  entry = (FAR struct work_notifier_entry_s *)arg;
  DEBUGASSERT(entry != NULL);

  ninfo = &entry->info;
  DEBUGASSERT(ninfo->arg != NULL);

  pinfo = (FAR struct tcp_poll_s *)ninfo->arg;
  DEBUGASSERT(pinfo->psock != NULL && pinfo->fds != NULL);

  psock = pinfo->psock;
  fds   = pinfo->fds;

  /* Verify that we still have a connection */

  if (!_SS_ISCONNECTED(psock->s_flags) && !_SS_ISLISTENING(psock->s_flags))
    {
      /* Don't report more than once.  Might happen in a race condition */

      if ((fds->revents & (POLLERR | POLLHUP)) == 0)
        {
          /* We were previously connected but lost the connection either due
           * to a graceful shutdown by the remote peer or because of some
           * exceptional event.
           */

          fds->revents |= (POLLERR | POLLHUP);
          nxsem_post(fds->sem);
        }
    }

  /* Handle a race condition.  Check if we have already posted the POLLOUT
   * event.  If so, don't do it again and don't setup notification again.
   */

  else if ((fds->events && POLLWRNORM) == 0 ||
           (fds->revents && POLLWRNORM) != 0)
    {
      /* Check if we are now able to send */

      if (psock_tcp_cansend(psock) >= 0)
        {
          /* Yes.. then signal the poll logic */

          fds->revents |= POLLWRNORM;
          nxsem_post(fds->sem);
        }
      else
        {
          /* No.. ask for the IOB free notification again */

          pinfo->key = iob_notifier_setup(LPWORK, tcp_iob_work, pinfo);
        }
    }

  /* Protocol for the use of the IOB notifier is that we free the argument
   * after the notification has been processed.
   */

  kmm_free(arg);
}
#endif

/****************************************************************************
 * Name: tcp_poll_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be sent (TCP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS) || !defined(CONFIG_IOB_NOTIFIER)
static inline void tcp_poll_txnotify(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;

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
#endif

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
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
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

  net_lock();

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
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  info->key    = 0;
#endif

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags    = (TCP_NEWDATA | TCP_BACKLOG | TCP_POLL | TCP_DISCONN_EVENTS);
  cb->priv     = (FAR void *)info;
  cb->event    = tcp_poll_eventhandler;

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

#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  /* If (1) revents == 0, (2) write buffering is enabled, and (3) the
   * POLLOUT event is needed, then setup to receive a notification an IOB
   * is freed.
   */

  else if ((fds->events & POLLOUT) != 0)
    {
      /* Ask for the IOB free notification */

      info->key = iob_notifier_setup(LPWORK, tcp_iob_work, info);
    }

#else
  /* If (1) the socket is in a bound state, (2) revents == 0, (3) write
   * buffering is not enabled (determined by a configuration setting), and
   * (4) the POLLOUT event is needed then request an immediate Tx poll from
   * the device associated with the binding.
   */

  else if (_SS_ISBOUND(psock->s_flags) && (fds->events & POLLOUT) != 0)
    {
      /* Note that the notification will fail if the socket is bound to
       * INADDR_ANY or the IPv6 unspecified address!  In that case the
       * notification will fail.
       */

      tcp_poll_txnotify(psock);
    }
#endif

  net_unlock();
  return OK;

errout_with_lock:
  kmm_free(info);
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
#if defined(CONFIG_NET_TCP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
      /* Cancel any pending IOB free notification */

      if (info->key > 0)
        {
          /* Ask for the IOB free notification */

          iob_notifier_teardown(info->key);
        }
#endif

      /* Release the callback */

      net_lock();
      tcp_callback_free(conn, info->cb);
      net_unlock();

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

#endif /* HAVE_TCP_POLL */
