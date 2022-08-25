/****************************************************************************
 * net/usrsock/usrsock_poll.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

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

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t poll_event(FAR struct net_driver_s *dev, FAR void *pvconn,
                           FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_poll_s *info = (FAR struct usrsock_poll_s *)pvpriv;
  FAR struct usrsock_conn_s *conn = info->conn;
  pollevent_t eventset = 0;

  DEBUGASSERT(!info || info->fds);

  if (info == NULL)
    {
      return flags;
    }

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

static int usrsock_pollsetup(FAR struct socket *psock,
                             FAR struct pollfd *fds)
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

  net_lock();

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->conn != NULL)
    {
      if (++info >= &conn->pollinfo[CONFIG_NET_USRSOCK_NPOLLWAITERS])
        {
          ret = -ENOMEM;
          goto errout_unlock;
        }
    }

  /* Allocate a usrsock callback structure */

  cb = devif_callback_alloc(NULL, &conn->sconn.list, &conn->sconn.list_tail);
  if (cb == NULL)
    {
      ret = -EBUSY;
      goto errout_unlock;
    }

  /* Initialize the poll info container */

  info->conn  = conn;
  info->fds   = fds;
  info->cb    = cb;

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

  else if ((conn->type == SOCK_STREAM ||
            conn->type == SOCK_SEQPACKET) &&
          !(conn->connected || conn->state ==
            USRSOCK_CONN_STATE_CONNECTING))
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

          fds->revents |= POLLOUT;
        }

      if (conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL)
        {
          ninfo("socket recv avail.\n");

          fds->revents |= POLLIN;
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
 *   fds   - The structure describing the events to be stopped being
 *           monitored.
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

      devif_conn_callback_free(NULL,
                               info->cb,
                               &conn->sconn.list,
                               &conn->sconn.list_tail);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      info->conn = NULL;
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

int usrsock_poll(FAR struct socket *psock,
                 FAR struct pollfd *fds, bool setup)
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

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
