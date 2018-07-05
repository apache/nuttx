/****************************************************************************
 * net/tcp/tcp_connect.c
 *
 *   Copyright (C) 2007-2012, 2015-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "socket/socket.h"
#include "tcp/tcp.h"

#ifdef NET_TCP_HAVE_STACK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tcp_connect_s
{
  FAR struct tcp_conn_s  *tc_conn;    /* Reference to TCP connection structure */
  FAR struct devif_callback_s *tc_cb; /* Reference to callback instance */
  FAR struct socket *tc_psock;        /* The socket being connected */
  sem_t tc_sem;                       /* Semaphore signals recv completion */
  int tc_result;                      /* OK on success, otherwise a negated errno. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int psock_setup_callbacks(FAR struct socket *psock,
                                        FAR struct tcp_connect_s *pstate);
static void psock_teardown_callbacks(FAR struct tcp_connect_s *pstate,
                                     int status);
static uint16_t psock_connect_eventhandler(FAR struct net_driver_s *dev,
                                           FAR void *pvconn,
                                           FAR void *pvpriv, uint16_t flags);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: psock_setup_callbacks
 ****************************************************************************/

static inline int psock_setup_callbacks(FAR struct socket *psock,
                                        FAR struct tcp_connect_s *pstate)
{
  FAR struct tcp_conn_s *conn = psock->s_conn;
  int ret = -EBUSY;

  /* Initialize the TCP state structure */

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  (void)nxsem_init(&pstate->tc_sem, 0, 0); /* Doesn't really fail */
  (void)nxsem_setprotocol(&pstate->tc_sem, SEM_PRIO_NONE);

  pstate->tc_conn   = conn;
  pstate->tc_psock  = psock;
  pstate->tc_result = -EAGAIN;

  /* Set up the callbacks in the connection */

  pstate->tc_cb = tcp_callback_alloc(conn);
  if (pstate->tc_cb)
    {
      /* Set up the connection event handler */

      pstate->tc_cb->flags   = (TCP_NEWDATA | TCP_CLOSE | TCP_ABORT |
                                TCP_TIMEDOUT | TCP_CONNECTED | NETDEV_DOWN);
      pstate->tc_cb->priv    = (FAR void *)pstate;
      pstate->tc_cb->event   = psock_connect_eventhandler;
      ret                    = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: psock_teardown_callbacks
 ****************************************************************************/

static void psock_teardown_callbacks(FAR struct tcp_connect_s *pstate,
                                     int status)
{
  FAR struct tcp_conn_s *conn = pstate->tc_conn;

  /* Make sure that no further events are processed */

  tcp_callback_free(conn, pstate->tc_cb);
  pstate->tc_cb = NULL;

  /* If we successfully connected, we will continue to monitor the connection
   * state via callbacks.
   */

  if (status < 0)
    {
      /* Failed to connect. Stop the connection event monitor */

      tcp_stop_monitor(conn, TCP_CLOSE);
    }
}

/****************************************************************************
 * Name: psock_connect_eventhandler
 *
 * Description:
 *   This function is called to perform the actual connection operation via
 *   by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that reported the event
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   The new flags setting
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t psock_connect_eventhandler(FAR struct net_driver_s *dev,
                                           FAR void *pvconn,
                                           FAR void *pvpriv, uint16_t flags)
{
  struct tcp_connect_s *pstate = (struct tcp_connect_s *)pvpriv;

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* The following errors should be detected here (someday)
       *
       *     ECONNREFUSED
       *       No one listening on the remote address.
       *     ENETUNREACH
       *       Network is unreachable.
       *     ETIMEDOUT
       *       Timeout while attempting connection. The server may be too busy
       *       to accept new connections.
       */

      /* TCP_CLOSE: The remote host has closed the connection
       * TCP_ABORT: The remote host has aborted the connection
       */

      if ((flags & (TCP_CLOSE | TCP_ABORT)) != 0)
        {
          /* Indicate that remote host refused the connection */

          pstate->tc_result = -ECONNREFUSED;
        }

      /* TCP_TIMEDOUT: Connection aborted due to too many retransmissions. */

      else if ((flags & TCP_TIMEDOUT) != 0)
        {
          /* Indicate that the connection timedout?) */

          pstate->tc_result = -ETIMEDOUT;
        }

      else if ((flags & NETDEV_DOWN) != 0)
        {
          /* The network device went down.  Indicate that the remote host
           * is unreachable.
           */

          pstate->tc_result = -ENETUNREACH;
        }

      /* TCP_CONNECTED: The socket is successfully connected */

      else if ((flags & TCP_CONNECTED) != 0)
        {
          FAR struct socket *psock = pstate->tc_psock;
          DEBUGASSERT(psock);

          /* Mark the connection bound and connected.  NOTE this is
           * is done here (vs. later) in order to avoid any race condition
           * in the socket state.  It is known to connected here and now,
           * but not necessarily at any time later.
           */

          psock->s_flags |= (_SF_BOUND | _SF_CONNECTED);

          /* Indicate that the socket is no longer connected */

          pstate->tc_result = OK;
        }

      /* Otherwise, it is not an event of importance to us at the moment */

      else
        {
          /* Drop data received in this state */

          dev->d_len = 0;
          return flags & ~TCP_NEWDATA;
        }

      ninfo("Resuming: %d\n", pstate->tc_result);

      /* Stop further callbacks */

      psock_teardown_callbacks(pstate, pstate->tc_result);

      /* We now have to filter all outgoing transfers so that they use only
       * the MSS of this device.
       */

      DEBUGASSERT(pstate->tc_conn != NULL);
      DEBUGASSERT(pstate->tc_conn->dev == NULL ||
                  pstate->tc_conn->dev == dev);
      pstate->tc_conn->dev = dev;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->tc_sem);
    }

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_tcp_connect
 *
 * Description:
 *   Perform a TCP connection
 *
 * Input Parameters:
 *   psock - A reference to the socket structure of the socket to be connected
 *   addr  - The address of the remote server to connect to
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

int psock_tcp_connect(FAR struct socket *psock,
                      FAR const struct sockaddr *addr)
{
  struct tcp_connect_s state;
  int                  ret = OK;

  /* Interrupts must be disabled through all of the following because
   * we cannot allow the network callback to occur until we are completely
   * setup.
   */

  net_lock();

  /* Get the connection reference from the socket */

  if (!psock->s_conn) /* Should always be non-NULL */
    {
      ret = -EINVAL;
    }
  else
    {
      /* Perform the TCP connection operation */

      ret = tcp_connect(psock->s_conn, addr);
    }

  if (ret >= 0)
    {
      /* Set up the callbacks in the connection */

      ret = psock_setup_callbacks(psock, &state);
      if (ret >= 0)
        {
          /* Wait for either the connect to complete or for an error/timeout
           * to occur. NOTES:  net_lockedwait will also terminate if a signal
           * is received.
           */

          ret = net_lockedwait(&state.tc_sem);

          /* Uninitialize the state structure */

          (void)nxsem_destroy(&state.tc_sem);

          /* If net_lockedwait failed, negated errno was returned. */

          if (ret >= 0)
            {
              /* If the wait succeeded, then get the new error value from
               * the state structure
               */

              ret = state.tc_result;
            }

          /* Make sure that no further events are processed */

          psock_teardown_callbacks(&state, ret);
        }

      /* Check if the socket was successfully connected. */

      if (ret >= 0)
        {
          /* Yes... Now that we are connected, we need to set up to monitor
           * the state of the connection up the connection event monitor.
           */

          ret = tcp_start_monitor(psock);
          if (ret < 0)
            {
              /* tcp_start_monitor() can only fail on certain race
               * conditions where the connection was lost just before
               * this function was called.  That is not expected to
               * happen in this context, but just in case...
               */

              tcp_stop_monitor(psock->s_conn, TCP_ABORT);
            }
        }
    }

  net_unlock();
  return ret;
}

#endif /* NET_TCP_HAVE_STACK */
