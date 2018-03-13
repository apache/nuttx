/****************************************************************************
 * net/usrsock/usrsock_connect.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
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

static uint16_t connect_event(FAR struct net_driver_s *dev, FAR void *pvconn,
                              FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pvconn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      pstate->result = -ECONNABORTED;

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ninfo("request completed.\n");

      pstate->result = conn->resp.result;

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_connect_request
 ****************************************************************************/

static int do_connect_request(FAR struct usrsock_conn_s *conn,
                              FAR const struct sockaddr *addr,
                              socklen_t addrlen)
{
  struct usrsock_request_connect_s req = {};
  struct iovec bufs[2];

  if (addrlen > UINT16_MAX)
    {
      addrlen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_CONNECT;
  req.usockid = conn->usockid;
  req.addrlen = addrlen;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);
  bufs[1].iov_base = (FAR void *)addr;
  bufs[1].iov_len = addrlen;

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_connect
 *
 * Description:
 *   Perform a usrsock connection
 *
 * Input Parameters:
 *   psock - A reference to the socket structure of the socket to be connected
 *   addr    The address of the remote server to connect to
 *   addrlen Length of address buffer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int usrsock_connect(FAR struct socket *psock,
                    FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_reqstate_s state = {};
  int ret;

  DEBUGASSERT(conn);

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; connect() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE : -ECONNREFUSED;
      goto errout_unlock;
    }

  if (conn->connected &&
      (conn->type == SOCK_STREAM || conn->type == SOCK_SEQPACKET))
    {
      /* Already connected. */

      ret = -EISCONN;
      goto errout_unlock;
    }

  if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
    {
      /* Already connecting. */

      ninfo("usockid=%d; socket already connecting.\n",
            conn->usockid);

      ret = -EALREADY;
      goto errout_unlock;
    }

  /* Set up event callback for usrsock. */

  ret = usrsock_setup_request_callback(conn, &state, connect_event,
                                       USRSOCK_EVENT_ABORT |
                                       USRSOCK_EVENT_REQ_COMPLETE);
  if (ret < 0)
    {
      nwarn("usrsock_setup_request_callback failed: %d\n", ret);

      goto errout_unlock;
    }

  /* Mark conn as connecting one. */

  conn->state = USRSOCK_CONN_STATE_CONNECTING;

  /* Send request. */

  ret = do_connect_request(conn, addr, addrlen);
  if (ret < 0)
    {
      goto errout_teardown;
    }

  /* Do not block on waiting for request completion if nonblocking socket. */

  if (!conn->resp.inprogress || !_SS_ISNONBLOCK(psock->s_flags))
    {
      /* Wait for completion of request (or signal). */

      ret = net_lockedwait(&state.recvsem);
      if (ret < 0)
        {
          DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);

          /* Wait interrupted, exit early. */

          goto errout_teardown;
        }

      ret = state.result;
    }
  else
    {
      /* Request not completed and socket is non-blocking. */

      ret = -EINPROGRESS;
    }

errout_teardown:
  usrsock_teardown_request_callback(&state);
errout_unlock:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
