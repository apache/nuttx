/****************************************************************************
 * net/socket/net_poll.c
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
#if defined(CONFIG_NET) && !defined(CONFIG_DISABLE_POLL)

#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/net/iob.h>
#include <nuttx/net/net.h>

#include <devif/devif.h>
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Network polling can only be supported on TCP and only if read-ahead buffering
 * is enabled (it could be supported on UDP as will if it also had read-ahead
 * buffering.
 */

#if !defined(CONFIG_DISABLE_POLL) && CONFIG_NSOCKET_DESCRIPTORS > 0 && \
    defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_READAHEAD)
#  define HAVE_NETPOLL 1
#else
#  undef HAVE_NETPOLL
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This is an allocated container that holds the poll-related information */

struct net_poll_s
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

#ifdef HAVE_NETPOLL
static uint16_t tcp_poll_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                                   FAR void *pvpriv, uint16_t flags)
{
  FAR struct net_poll_s *info = (FAR struct net_poll_s *)pvpriv;

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

      if ((flags & (TCP_CLOSE | TCP_ABORT | TCP_TIMEDOUT)) != 0)
        {
          /* Marki that the connection has been lost */

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
#endif /* HAVE_NETPOLL */

/****************************************************************************
 * Function: udp_poll_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   UDP receive operation via by the device interface layer.
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

#ifdef HAVE_NETPOLL
static uint16_t udp_poll_interrupt(FAR struct net_driver_s *dev, FAR void *conn,
                                   FAR void *pvpriv, uint16_t flags)
{
  FAR struct net_poll_s *info = (FAR struct net_poll_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  DEBUGASSERT(!info || (info->psock && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & (UDP_NEWDATA)) != 0)
        {
          eventset |= (POLLIN & info->fds->events);
        }

      /* A poll is a sign that we are free to send data. */

      if ((flags & UDP_POLL) != 0)
        {
          eventset |= (POLLOUT & info->fds->events);
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
#endif /* HAVE_NETPOLL */

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

#ifdef HAVE_NETPOLL
static inline int tcp_pollsetup(FAR struct socket *psock,
                                FAR struct pollfd *fds)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  FAR struct net_poll_s *info;
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

  info = (FAR struct net_poll_s *)kmm_malloc(sizeof(struct net_poll_s));
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

  cb->flags    = (TCP_NEWDATA | TCP_BACKLOG | TCP_POLL | TCP_CLOSE |
                  TCP_ABORT | TCP_TIMEDOUT);
  cb->priv     = (FAR void *)info;
  cb->event    = tcp_poll_interrupt;

  /* Save the reference in the poll info structure as fds private as well
   * for use durring poll teardown as well.
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
 * Function: udp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one UDP/IP socket
 *
 * Input Parameters:
 *   psock - The UDP/IP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static inline int udp_pollsetup(FAR struct socket *psock,
                                FAR struct pollfd *fds)
{
  FAR struct udp_conn_s *conn = psock->s_conn;
  FAR struct net_poll_s *info;
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

  info = (FAR struct net_poll_s *)kmm_malloc(sizeof(struct net_poll_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Some of the  following must be atomic */

  flags = net_lock();

  /* Setup the UDP remote connection */

  ret = udp_connect(conn, NULL);
  if (ret)
    {
      goto errout_with_lock;
    }

  /* Allocate a TCP/IP callback structure */

  cb = udp_callback_alloc(conn);
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

  cb->flags    = (0);
  cb->priv     = (FAR void *)info;
  cb->event    = udp_poll_interrupt;

  if (info->fds->events & POLLOUT)
    cb->flags |= UDP_POLL;
  if (info->fds->events & POLLIN)
    cb->flags |= UDP_NEWDATA;

  /* Save the reference in the poll info structure as fds private as well
   * for use durring poll teardown as well.
   */

  fds->priv    = (FAR void *)info;

  /* Check for read data availability now */

  if (!IOB_QEMPTY(&conn->readahead))
    {
      /* Normal data may be read without blocking. */

      fds->revents |= (POLLRDNORM & fds->events);
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
 * Function: net_pollsetup
 *
 * Description:
 *   Setup to monitor events on one socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static inline int net_pollsetup(FAR struct socket *psock,
                                FAR struct pollfd *fds)
{
#ifdef CONFIG_NET_TCP
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollsetup(psock, fds);
    }
#endif

#ifdef CONFIG_NET_UDP
  if (psock->s_type != SOCK_STREAM)
    {
      return udp_pollsetup(psock, fds);
    }
#endif

  return -ENOSYS;
}
#endif /* HAVE_NETPOLL */

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

#ifdef HAVE_NETPOLL
static inline int tcp_pollteardown(FAR struct socket *psock,
                                   FAR struct pollfd *fds)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  FAR struct net_poll_s *info;
  net_lock_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct net_poll_s *)fds->priv;
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

/****************************************************************************
 * Function: udp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an UDP/IP socket
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

static inline int udp_pollteardown(FAR struct socket *psock,
                                   FAR struct pollfd *fds)
{
  FAR struct udp_conn_s *conn = psock->s_conn;
  FAR struct net_poll_s *info;
  net_lock_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct net_poll_s *)fds->priv;
  DEBUGASSERT(info && info->fds && info->cb);
  if (info)
    {
      /* Release the callback */

      flags = net_lock();
      udp_callback_free(conn, info->cb);
      net_unlock(flags);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

/****************************************************************************
 * Function: net_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an socket
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

static inline int net_pollteardown(FAR struct socket *psock,
                                   FAR struct pollfd *fds)
{
#ifdef CONFIG_NET_TCP
  if (psock->s_type == SOCK_STREAM)
    {
      return tcp_pollteardown(psock, fds);
    }
#endif

#ifdef CONFIG_NET_UDP
  if (psock->s_type != SOCK_STREAM)
    {
      return udp_pollteardown(psock, fds);
    }
#endif

  return -ENOSYS;
}
#endif /* HAVE_NETPOLL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_POLL) && defined(HAVE_NETPOLL)
int psock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup)
{
  int ret;

  /* Check if we are setting up or tearing down the poll */

  if (setup)
    {
      /* Perform the TCP/IP poll() setup */

      ret = net_pollsetup(psock, fds);
    }
  else
    {
      /* Perform the TCP/IP poll() teardown */

      ret = net_pollteardown(psock, fds);
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: net_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   fd    - The socket descriptor of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
int net_poll(int sockfd, struct pollfd *fds, bool setup)
{
#ifndef HAVE_NETPOLL
  return -ENOSYS;
#else
  FAR struct socket *psock;

  /* Get the underlying socket structure and verify that the sockfd
   * corresponds to valid, allocated socket
   */

  psock = sockfd_socket(sockfd);
  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Then let psock_poll() do the heavy lifting */

  return psock_poll(psock, fds, setup);
#endif /* HAVE_NETPOLL */
}
#endif /* !CONFIG_DISABLE_POLL */

#endif /* CONFIG_NET && !CONFIG_DISABLE_POLL */
