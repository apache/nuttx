/****************************************************************************
 * net/icmpv6/icmpv6_netpoll.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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
#include "netdev/netdev.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is an allocated container that holds the poll-related information */

struct icmpv6_poll_s
{
  FAR struct socket *psock;        /* IPPROTO_ICMP6 socket structure */
  FAR struct pollfd *fds;          /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_poll_eventhandler
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

static uint16_t icmpv6_poll_eventhandler(FAR struct net_driver_s *dev,
                                         FAR void *pvconn,
                                         FAR void *pvpriv, uint16_t flags)
{
  FAR struct icmpv6_poll_s *info = (FAR struct icmpv6_poll_s *)pvpriv;
  FAR struct icmpv6_conn_s *conn;
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
      if ((flags & ICMPv6_ECHOREPLY) != 0)
        {
          eventset |= (POLLIN & info->fds->events);
        }

      /*  ICMP_POLL is a sign that we are free to send data. */

      if ((flags & DEVPOLL_MASK) == ICMPv6_POLL)
        {
          eventset |= (POLLOUT & info->fds->events);
        }

      /* Check for loss of connection events. */

      if ((flags & NETDEV_DOWN) != 0)
        {
          eventset |= ((POLLHUP | POLLERR) & info->fds->events);
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
 * Name: icmpv6_pollsetup
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

int icmpv6_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct icmpv6_conn_s *conn = psock->s_conn;
  FAR struct icmpv6_poll_s *info;
  FAR struct devif_callback_s *cb;
  int ret;

  DEBUGASSERT(conn != NULL && fds != NULL);

  /* Allocate a container to hold the poll information */

  info = (FAR struct icmpv6_poll_s *)kmm_malloc(sizeof(struct icmpv6_poll_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Some of the  following must be atomic */

  net_lock();

  /* Get the device that will provide the provide the NETDEV_DOWN event.
   * NOTE: in the event that the local socket is bound to IN6ADDR_ANY, the
   * dev value will be zero and there will be no NETDEV_DOWN notifications.
   */

  /* Allocate a ICMP callback structure */

  if (conn->dev == NULL)
    {
      conn->dev = netdev_default();
    }

  cb = icmpv6_callback_alloc(conn->dev);
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

  cb->flags = 0;
  cb->priv  = (FAR void *)info;
  cb->event = icmpv6_poll_eventhandler;

  if ((info->fds->events & POLLOUT) != 0)
    {
      cb->flags |= ICMPv6_POLL;
    }

  if ((info->fds->events & POLLIN) != 0)
    {
      cb->flags |= ICMPv6_NEWDATA;
    }

  if ((info->fds->events & (POLLHUP | POLLERR)) != 0)
    {
      cb->flags |= NETDEV_DOWN;
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

  /* Check if any requested events are already in effect */

  if (fds->revents != 0)
    {
      /* Yes.. then signal the poll logic */

      nxsem_post(fds->sem);
    }

  net_unlock();
  return OK;

errout_with_lock:
  kmm_free(info);
  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: icmpv6_pollteardown
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

int icmpv6_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct icmpv6_conn_s *conn;
  FAR struct icmpv6_poll_s *info;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL &&
              fds != NULL && fds->priv != NULL);

  conn = psock->s_conn;

  /* Recover the socket descriptor poll state info from the poll structure */

  info = (FAR struct icmpv6_poll_s *)fds->priv;
  DEBUGASSERT(info != NULL && info->fds != NULL && info->cb != NULL);

  if (info != NULL)
    {
      /* Release the callback */

      net_lock();
      icmpv6_callback_free(conn->dev, info->cb);
      net_unlock();

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

#endif /* !CONFIG_MM_IOB */
