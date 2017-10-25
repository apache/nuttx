/****************************************************************************
 * net/usrsock/usrsock_poll.c
 *
 *  Copyright (C) 2015, 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK) && \
    !defined(CONFIG_DISABLE_POLL)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>
#include <nuttx/kmalloc.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct usrsock_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  struct pollfd *fds;              /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t poll_event(FAR struct net_driver_s *dev, FAR void *pvconn,
                           FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_poll_s *info = (FAR struct usrsock_poll_s *)pvpriv;
  FAR struct usrsock_conn_s *conn = pvconn;
  pollevent_t eventset = 0;

  DEBUGASSERT(!info || (info->psock && info->fds));

  if (!info)
    return flags;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      /* Socket forcefully terminated. */

      eventset |= (POLLERR | POLLHUP);
    }
  else if ((flags & USRSOCK_EVENT_CONNECT_READY) && !conn->connected)
    {
      ninfo("socket connect failed.\n");

      /* Non-blocking connect failed. */

      eventset |= (POLLERR | POLLHUP);
    }
  else if (flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("remote closed.\n");

      /* Remote closed. */

      eventset |= (POLLHUP | POLLIN);
    }
  else
    {
      /* Check data events. */

      if (flags & USRSOCK_EVENT_RECVFROM_AVAIL)
        {
          ninfo("socket recv avail.\n");

          eventset |= POLLIN;
        }

      if (flags & USRSOCK_EVENT_SENDTO_READY)
        {
          ninfo("socket send ready.\n");

          eventset |= POLLOUT;
        }
    }

  /* Filter I/O events depending on requested events. */

  eventset &= (~(POLLOUT | POLLIN) | info->fds->events);

  /* POLLOUT and PULLHUP are mutually exclusive. */

  if ((eventset & POLLOUT) && (eventset & POLLHUP))
    {
      eventset &= ~POLLOUT;
    }

  /* Awaken the caller of poll() is requested event occurred. */

  if (eventset)
    {
      info->fds->revents |= eventset;
      nxsem_post(info->fds->sem);
    }

  return flags;
}

/****************************************************************************
 * Name: usrsock_poll
 *
 * Description:
 *   Setup to monitor events on an usrsock socket
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int usrsock_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  FAR struct usrsock_poll_s *info;
  FAR struct devif_callback_s *cb;
  int ret = OK;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a container to hold the poll information */

  info = (FAR struct usrsock_poll_s *)kmm_malloc(sizeof(struct usrsock_poll_s));
  if (!info)
    {
      return -ENOMEM;
    }

  net_lock();

  /* Allocate a usrsock callback structure */

  cb = devif_callback_alloc(NULL, &conn->list);
  if (cb == NULL)
    {
      ret = -EBUSY;
      kmm_free(info); /* fds->priv not set, so we need to free info here. */
      goto errout_unlock;
    }

  /* Initialize the poll info container */

  info->psock  = psock;
  info->fds    = fds;
  info->cb     = cb;

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags    = USRSOCK_EVENT_ABORT | USRSOCK_EVENT_CONNECT_READY |
                 USRSOCK_EVENT_SENDTO_READY | USRSOCK_EVENT_RECVFROM_AVAIL |
                 USRSOCK_EVENT_REMOTE_CLOSED;
  cb->priv     = (FAR void *)info;
  cb->event    = poll_event;

  /* Save the reference in the poll info structure as fds private as well
   * for use during poll teardown as well.
   */

  fds->priv    = (FAR void *)info;

  /* Check if socket is in error state */

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      ninfo("socket %s.\n",
            conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ?
                "uninitialized" : "aborted");

      fds->revents |= (POLLERR | POLLHUP);
    }

  /* Stream sockets need to be connected or connecting (or listening). */

  else if ((conn->type == SOCK_STREAM || conn->type == SOCK_SEQPACKET) &&
           !(conn->connected || conn->state == USRSOCK_CONN_STATE_CONNECTING))
    {
      ninfo("stream socket not connected and not connecting.\n");

      fds->revents |= (POLLOUT | POLLIN | POLLHUP);
    }
  else if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("socket remote closed.\n");

      /* Remote closed. */

      fds->revents |= (POLLHUP | POLLIN);
    }
  else
    {
      /* Check if daemon has room for send data or has data to receive. */

      if (conn->flags & USRSOCK_EVENT_SENDTO_READY)
        {
          ninfo("socket send ready.\n");

          fds->revents |= (POLLOUT & fds->events);
        }

      if (conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL)
        {
          ninfo("socket recv avail.\n");

          fds->revents |= (POLLIN & fds->events);
        }
    }

  /* Filter I/O events depending on requested events. */

  fds->revents &= (~(POLLOUT | POLLIN) | info->fds->events);

  /* POLLOUT and PULLHUP are mutually exclusive. */

  if ((fds->revents & POLLOUT) && (fds->revents & POLLHUP))
    {
      fds->revents &= ~POLLOUT;
    }

  /* Check if any requested events are already in effect */

  if (fds->revents != 0)
    {
      /* Yes.. then signal the poll logic */

      nxsem_post(fds->sem);
    }

errout_unlock:
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: usrsock_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an usrsock socket
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be stopped being monitored.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int usrsock_pollteardown(FAR struct socket *psock,
                                FAR struct pollfd *fds)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  FAR struct usrsock_poll_s *info;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct usrsock_poll_s *)fds->priv;
  DEBUGASSERT(info && info->fds && info->cb);
  if (info)
    {
      /* Release the callback */

      net_lock();
      devif_conn_callback_free(NULL, info->cb, &conn->list);
      net_unlock();

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   fds   - The structure describing the events to be monitored.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int usrsock_poll(FAR struct socket *psock, FAR struct pollfd *fds, bool setup)
{
  if (setup)
    {
      return usrsock_pollsetup(psock, fds);
    }
  else
    {
      return usrsock_pollteardown(psock, fds);
    }
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK && !CONFIG_DISABLE_POLL */
