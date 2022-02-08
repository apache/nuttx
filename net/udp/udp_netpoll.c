/****************************************************************************
 * net/udp/udp_netpoll.c
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
#include <nuttx/semaphore.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "udp/udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual UDP receive operation
 *   via the device interface layer.
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
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static uint16_t udp_poll_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *conn,
                                      FAR void *pvpriv, uint16_t flags)
{
  FAR struct udp_poll_s *info = (FAR struct udp_poll_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(!info || (info->conn && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & UDP_NEWDATA) != 0)
        {
          eventset |= (POLLIN & info->fds->events);
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= (POLLHUP | POLLERR);
        }

      /* A poll is a sign that we are free to send data. */

      else if (psock_udp_cansend(info->conn) >= 0)
        {
          eventset |= (POLLOUT & info->fds->events);
        }

      /* Awaken the caller of poll() is requested event occurred. */

      if (eventset)
        {
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
 * Name: udp_pollsetup
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

int udp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct udp_conn_s *conn = psock->s_conn;
  FAR struct udp_poll_s *info;
  FAR struct devif_callback_s *cb;
  int ret = OK;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (conn == NULL || fds == NULL)
    {
      return -EINVAL;
    }
#endif

  /* Some of the  following must be atomic */

  net_lock();

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->conn != NULL)
    {
      if (++info >= &conn->pollinfo[CONFIG_NET_UDP_NPOLLWAITERS])
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }
    }

  /* Get the device that will provide the provide the NETDEV_DOWN event.
   * NOTE: in the event that the local socket is bound to INADDR_ANY, the
   * dev value will be zero and there will be no NETDEV_DOWN notifications.
   */

  info->dev = udp_find_laddr_device(conn);

  /* Allocate a UDP callback structure */

  cb = udp_callback_alloc(info->dev, conn);
  if (cb == NULL)
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

  cb->flags    = NETDEV_DOWN;
  cb->priv     = (FAR void *)info;
  cb->event    = udp_poll_eventhandler;

  if ((fds->events & POLLOUT) != 0)
    {
      cb->flags |= UDP_POLL;
    }

  if ((fds->events & POLLIN) != 0)
    {
      cb->flags |= UDP_NEWDATA;
    }

  /* Save the reference in the poll info structure as fds private as well
   * for use during poll teardown as well.
   */

  fds->priv = (FAR void *)info;

  /* Check for read data availability now */

  if (!IOB_QEMPTY(&conn->readahead))
    {
      /* Normal data may be read without blocking. */

      fds->revents |= (POLLRDNORM & fds->events);
    }

  if (psock_udp_cansend(conn) >= 0)
    {
      /* Normal data may be sent without blocking (at least one byte). */

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
 * Name: udp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an UDP/IP socket
 *
 * Input Parameters:
 *   psock - The UDP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int udp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct udp_conn_s *conn = psock->s_conn;
  FAR struct udp_poll_s *info;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }
#endif

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct udp_poll_s *)fds->priv;
  DEBUGASSERT(info != NULL && info->fds != NULL && info->cb != NULL);
  if (info != NULL)
    {
      /* Release the callback */

      udp_callback_free(info->dev, conn, info->cb);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      info->conn = NULL;
    }

  return OK;
}
