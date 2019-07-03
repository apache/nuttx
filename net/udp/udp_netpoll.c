/****************************************************************************
 * net/udp/udp_netpoll.c
 *
 *   Copyright (C) 2008-2009, 2011-2015, 2018 Gregory Nutt. All rights
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
#include "udp/udp.h"

#ifdef HAVE_UDP_POLL

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is an allocated container that holds the poll-related information */

struct udp_poll_s
{
  FAR struct socket *psock;        /* Needed to handle loss of connection */
  FAR struct net_driver_s *dev;    /* Needed to free the callback structure */
  struct pollfd *fds;              /* Needed to handle poll events */
  FAR struct devif_callback_s *cb; /* Needed to teardown the poll */
#if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  int16_t key;                     /* Needed to cancel pending notification */
#endif
};

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

  DEBUGASSERT(!info || (info->psock && info->fds));

  /* 'priv' might be null in some race conditions (?) */

  if (info)
    {
      pollevent_t eventset = 0;

      /* Check for data or connection availability events. */

      if ((flags & UDP_NEWDATA) != 0)
        {
          eventset |= (POLLIN & info->fds->events);
        }

      /* A poll is a sign that we are free to send data.
       * REVISIT: This is bogus:  If CONFIG_UDP_WRITE_BUFFERS=y then
       * we never have to wait to send; otherwise, we always have to
       * wait to send.  Receiving a poll is irrelevant.
       */

      if ((flags & UDP_POLL) != 0)
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
 * Name: udp_iob_work
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

#if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
static inline void udp_iob_work(FAR void *arg)
{
  FAR struct work_notifier_entry_s *entry;
  FAR struct work_notifier_s *ninfo;
  FAR struct udp_poll_s *pinfo;
  FAR struct socket *psock;
  FAR struct pollfd *fds;

  entry = (FAR struct work_notifier_entry_s *)arg;
  DEBUGASSERT(entry != NULL);

  ninfo = &entry->info;
  DEBUGASSERT(ninfo->arg != NULL);

  pinfo = (FAR struct udp_poll_s *)ninfo->arg;
  DEBUGASSERT(pinfo->psock != NULL && pinfo->fds != NULL);

  psock = pinfo->psock;
  fds   = pinfo->fds;

  /* Handle a race condition.  Check if we have already posted the POLLOUT
   * event.  If so, don't do it again.
   */

  if ((fds->events && POLLWRNORM) == 0 ||
      (fds->revents && POLLWRNORM) != 0)
    {
      /* Check if we are now able to send */

      if (psock_udp_cansend(psock) >= 0)
        {
          /* Yes.. then signal the poll logic */

          fds->revents |= POLLWRNORM;
          nxsem_post(fds->sem);
        }
      else
        {
          /* No.. ask for the IOB free notification again */

          pinfo->key = iob_notifier_setup(LPWORK, udp_iob_work, pinfo);
        }
    }

  /* Protocol for the use of the IOB notifier is that we free the argument
   * after the notification has been processed.
   */

  kmm_free(arg);
}
#endif

/****************************************************************************
 * Name: udp_poll_txnotify
 *
 * Description:
 *   Notify the appropriate device driver that we are have data ready to
 *   be sent (UDP)
 *
 * Input Parameters:
 *   psock - Socket state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_NET_UDP_WRITE_BUFFERS) || !defined(CONFIG_IOB_NOTIFIER)
static inline void udp_poll_txnotify(FAR struct socket *psock)
{
  FAR struct udp_conn_s *conn = psock->s_conn;

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
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (conn == NULL || fds == NULL)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a container to hold the poll information */

  info = (FAR struct udp_poll_s *)kmm_malloc(sizeof(struct udp_poll_s));
  if (!info)
    {
      return -ENOMEM;
    }

  /* Some of the  following must be atomic */

  net_lock();

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

  info->psock  = psock;
  info->fds    = fds;
  info->cb     = cb;
#if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  info->key    = 0;
#endif

  /* Initialize the callback structure.  Save the reference to the info
   * structure as callback private data so that it will be available during
   * callback processing.
   */

  cb->flags    = 0;
  cb->priv     = (FAR void *)info;
  cb->event    = udp_poll_eventhandler;

  if ((info->fds->events & POLLOUT) != 0)
    {
      cb->flags |= UDP_POLL;
    }

  if ((info->fds->events & POLLIN) != 0)
    {
      cb->flags |= UDP_NEWDATA;
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

  if (psock_udp_cansend(psock) >= 0)
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

#if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
  /* If (1) revents == 0, (2) write buffering is enabled, and (3) the
   * POLLOUT event is needed, then setup to receive a notification an IOB
   * is freed.
   */

  else if ((fds->events & POLLOUT) != 0)
    {
      /* Ask for the IOB free notification */

      info->key = iob_notifier_setup(LPWORK, udp_iob_work, info);
    }

#else
  /* If (1) the socket is in a bound state via bind() or via the
   * UDP_BINDTODEVICE socket options, (2) revents == 0, (3) write buffering
   * is not enabled (determined by a configuration setting), and (3) the
   * POLLOUT event is needed then request an immediate Tx poll from the
   * device associated with the binding.
   */

  else if ((fds->events & POLLOUT) != 0)
    {
      /* Check if the socket has been bound to a local address (might be
       * INADDR_ANY or the IPv6 unspecified address!  In that case the
       * notification will fail)
       */

      if (_SS_ISBOUND(psock->s_flags))
        {
          udp_poll_txnotify(psock);
        }

#ifdef CONFIG_NET_UDP_BINDTODEVICE
      /* Check if the socket has been bound to a device interface index via
       * the UDP_BINDTODEVICE socket option.
       */

      else if (conn->boundto > 0)
        {
          /* Yes, find the device associated with the interface index */

          FAR struct net_driver_s *dev = netdev_findbyindex(conn->boundto);
          if (dev != NULL)
            {
              /* And request a poll from the device */

              netdev_txnotify_dev(dev);
            }
        }
#endif
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
     #if defined(CONFIG_NET_UDP_WRITE_BUFFERS) && defined(CONFIG_IOB_NOTIFIER)
      /* Cancel any pending IOB free notification */

      if (info->key > 0)
        {
          /* Ask for the IOB free notification */

          iob_notifier_teardown(info->key);
        }
#endif

      /* Release the callback */

      net_lock();
      udp_callback_free(info->dev, conn, info->cb);
      net_unlock();

      /* Release the poll/select data slot */

      info->fds->priv = NULL;

      /* Then free the poll info container */

      kmm_free(info);
    }

  return OK;
}

#endif /* !HAVE_UDP_POLL */
