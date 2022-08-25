/****************************************************************************
 * net/icmp/icmp_netpoll.c
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
#include <poll.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "icmp/icmp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_poll_eventhandler
 *
 * Description:
 *   This function is called to perform the actual UDP receive operation
 *   via the device interface layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that caused the event
 *   pvpriv   An instance of struct icmp_poll_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

static uint16_t icmp_poll_eventhandler(FAR struct net_driver_s *dev,
                                       FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmp_poll_s *info = pvpriv;
  FAR struct icmp_conn_s *conn;
  FAR struct socket *psock;
  pollevent_t eventset;

  ninfo("flags: %04x\n", flags);

  DEBUGASSERT(info == NULL || info->fds != NULL);

  /* 'priv' might be null in some race conditions (?).  Only process the
   * the event if this poll is from the same device that the request was
   * sent out on.
   */

  if (info != NULL)
    {
      /* Is this a response on the same device that we sent the request out
       * on?
       */

      psock = info->psock;
      DEBUGASSERT(psock != NULL && psock->s_conn != NULL);
      conn  = psock->s_conn;
      if (dev != conn->dev)
        {
          ninfo("Wrong device\n");
          return flags;
        }

      /* Check for data or connection availability events. */

      eventset = 0;
      if ((flags & ICMP_NEWDATA) != 0)
        {
          eventset |= (POLLIN & info->fds->events);
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= (POLLHUP | POLLERR);
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
 * Name: icmp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int icmp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct icmp_conn_s *conn;
  FAR struct icmp_poll_s *info;
  FAR struct devif_callback_s *cb;
  int ret = OK;

  /* Some of the following must be atomic */

  net_lock();

  conn = psock->s_conn;

  /* Sanity check */

  if (!conn || !fds)
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Find a container to hold the poll information */

  info = conn->pollinfo;
  while (info->psock != NULL)
    {
      if (++info >= &conn->pollinfo[CONFIG_NET_ICMP_NPOLLWAITERS])
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }
    }

  /* Allocate a ICMP callback structure */

  cb = icmp_callback_alloc(conn->dev, conn);
  if (cb == NULL)
    {
      ret = -EBUSY;
      goto errout_with_lock;
    }

  /* Initialize the poll info container */

  info->psock = psock;
  info->fds   = fds;
  info->cb    = cb;

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags = NETDEV_DOWN;
  cb->priv  = (FAR void *)info;
  cb->event = icmp_poll_eventhandler;

  if ((fds->events & POLLIN) != 0)
    {
      cb->flags |= ICMP_NEWDATA;
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

  /* Always report POLLWRNORM if caller request it because we don't utilize
   * IOB buffer for sending.
   */

  fds->revents |= (POLLWRNORM & fds->events);

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
 * Name: icmp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int icmp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct icmp_conn_s *conn;
  FAR struct icmp_poll_s *info;

  /* Some of the following must be atomic */

  net_lock();

  conn = psock->s_conn;

  /* Sanity check */

  if (!conn || !fds->priv)
    {
      net_unlock();
      return -EINVAL;
    }

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct icmp_poll_s *)fds->priv;
  DEBUGASSERT(info->fds != NULL && info->cb != NULL);

  if (info != NULL)
    {
      /* Release the callback */

      icmp_callback_free(conn->dev, conn, info->cb);

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      info->psock = NULL;
    }

  net_unlock();

  return OK;
}
