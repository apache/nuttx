/****************************************************************************
 * net/pkt/pkt_netpoll.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/net/pkt.h>
#include <nuttx/semaphore.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "socket/socket.h"
#include "utils/utils.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_pkt_cansend
 *
 * Description:
 *   psock_pkt_cansend() returns a value indicating if a write to the socket
 *   would block.  It is still possible that the write may block if another
 *   write occurs first.
 *
 * Input Parameters:
 *   conn     A reference to PKT connection structure.
 *
 * Returned Value:
 *   OK (Always can send).
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static int psock_pkt_cansend(FAR struct pkt_conn_s *conn)
{
  if (iob_navail(false) <= 0
#if defined(CONFIG_NET_PKT_WRITE_BUFFERS) && CONFIG_NET_SEND_BUFSIZE > 0
      || iob_get_queue_size(&conn->write_q) >= conn->sndbufs
#endif
     )
    {
      return -EWOULDBLOCK;
    }

  return OK;
}

/****************************************************************************
 * Name: pkt_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual PKT receive operation via
 *   the device interface layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct pkt_poll_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static uint16_t pkt_poll_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvpriv, uint16_t flags)
{
  FAR struct pkt_poll_s *info = pvpriv;

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(!info || (info->conn && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data availability events. */

      if ((flags & PKT_NEWDATA) != 0)
        {
          eventset |= POLLIN;
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= (POLLHUP | POLLERR);
        }

      /* A poll is a sign that we are free to send data. */

      else if (psock_pkt_cansend(info->conn) >= 0)
        {
          eventset |= POLLOUT;
        }

      /* Awaken the caller of poll() is requested event occurred. */

      poll_notify(&info->fds, 1, eventset);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_pollsetup
 *
 * Description:
 *   Setup to monitor events on one PKT socket
 *
 * Input Parameters:
 *   psock - The PKT socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int pkt_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct pkt_conn_s *conn;
  FAR struct pkt_poll_s *info;
  FAR struct devif_callback_s *cb;
  FAR struct net_driver_s *dev;
  pollevent_t eventset = 0;
  int ret = OK;

  /* Some of the following must be atomic */

  conn = psock->s_conn;

  /* Sanity check */

  if (conn == NULL || fds == NULL)
    {
      return -EINVAL;
    }

  dev = pkt_find_device(conn);
  if (dev == NULL)
    {
      nerr("ERROR: No device found for PKT connection\n");
      return -ENODEV;
    }

  conn_dev_lock(&conn->sconn, dev);

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->conn != NULL)
    {
      if (++info >= &conn->pollinfo[CONFIG_NET_PKT_NPOLLWAITERS])
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }
    }

  /* Allocate a PKT callback structure */

  cb = pkt_callback_alloc(dev, conn);
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

  cb->flags = NETDEV_DOWN;
  cb->priv  = info;
  cb->event = pkt_poll_eventhandler;

  if ((fds->events & POLLOUT) != 0)
    {
      cb->flags |= PKT_POLL;
    }

  if ((fds->events & POLLIN) != 0)
    {
      cb->flags |= PKT_NEWDATA;
    }

  /* Save the reference in the poll info structure as fds private as well
   * for use during poll teardown as well.
   */

  fds->priv = info;

  /* Check for read data availability now */

  if (iob_peek_queue(&conn->readahead) != NULL)
    {
      /* Normal data may be read without blocking. */

      eventset |= POLLRDNORM;
    }

  /* Check for write data availability now */

  if (psock_pkt_cansend(conn) >= 0)
    {
      /* Normal data may be sent without blocking (at least one byte). */

      eventset |= POLLWRNORM;
    }

  /* Check if any requested events are already in effect */

  poll_notify(&fds, 1, eventset);

errout_with_lock:
  conn_dev_unlock(&conn->sconn, dev);
  return ret;
}

/****************************************************************************
 * Name: pkt_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an PKT socket
 *
 * Input Parameters:
 *   psock - The PKT socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int pkt_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct pkt_conn_s   *conn;
  FAR struct pkt_poll_s   *info;
  FAR struct net_driver_s *dev;

  conn = psock->s_conn;

  /* Sanity check */

  if (!conn || !fds->priv)
    {
      return -EINVAL;
    }

  dev = pkt_find_device(conn);
  if (dev == NULL)
    {
      nerr("ERROR: No device found for PKT connection\n");
      return -ENODEV;
    }

  conn_dev_lock(&conn->sconn, dev);

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct pkt_poll_s *)fds->priv;
  DEBUGASSERT(info->fds != NULL && info->cb != NULL);
  if (info != NULL)
    {
      /* Release the callback */

      pkt_callback_free(dev, conn, info->cb);

      /* Release the poll/select data slot */

      fds->priv = NULL;

      /* Then free the poll info container */

      info->conn = NULL;
    }

  conn_dev_unlock(&conn->sconn, dev);

  return OK;
}
