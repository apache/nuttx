/****************************************************************************
 * nuttx/net/usrsock/usrsock_ioctl.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Jianli Dong<dongjianli@pinecone.net>
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

#include <sys/socket.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "socket/socket.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t ioctl_event(FAR struct net_driver_s *dev,
                                  FAR void *pvconn,
                                  FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_data_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pvconn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      pstate->reqstate.result = -ECONNABORTED;
      pstate->valuelen = 0;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ninfo("request completed.\n");

      pstate->reqstate.result = conn->resp.result;
      if (pstate->reqstate.result < 0)
        {
          pstate->valuelen = 0;
          pstate->valuelen_nontrunc = 0;
        }
      else
        {
          pstate->valuelen = conn->resp.valuelen;
          pstate->valuelen_nontrunc = conn->resp.valuelen_nontrunc;
        }

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_ioctl_request
 ****************************************************************************/

static int do_ioctl_request(FAR struct usrsock_conn_s *conn, int cmd,
                                 FAR void *arg, size_t arglen)
{
  struct usrsock_request_ioctl_s req = {};
  struct iovec bufs[2];

  if (arglen > UINT16_MAX)
    {
      arglen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_IOCTL;
  req.usockid = conn->usockid;
  req.cmd = cmd;
  req.arglen = arglen;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);
  bufs[1].iov_base = (FAR void *)arg;
  bufs[1].iov_len = req.arglen;

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_ioctl
 *
 * Description:
 *   The usrsock_ioctl() function performs network device specific operations.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 ****************************************************************************/

int usrsock_ioctl(FAR struct socket *psock, int cmd, FAR void *arg, size_t arglen)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state = {};
  struct iovec inbufs[1];
  int ret;

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; ioctl() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE : -ECONNRESET;
      goto errout_unlock;
    }

  /* Set up event callback for usrsock. */

  ret = usrsock_setup_data_request_callback(
      conn, &state, ioctl_event,
      USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
  if (ret < 0)
    {
      nwarn("usrsock_setup_request_callback failed: %d\n", ret);
      goto errout_unlock;
    }

  inbufs[0].iov_base = arg;
  inbufs[0].iov_len = arglen;
  usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

  /* Request user-space daemon to close socket. */

  ret = do_ioctl_request(conn, cmd, arg, arglen);
  if (ret >= 0)
    {
      /* Wait for completion of request. */

      while ((ret = net_lockedwait(&state.reqstate.recvsem)) < 0)
        {
          DEBUGASSERT(ret == -EINTR);
        }

      ret = state.reqstate.result;

      DEBUGASSERT(state.valuelen <= arglen);
      DEBUGASSERT(state.valuelen <= state.valuelen_nontrunc);
    }

  usrsock_teardown_datain(conn);
  usrsock_teardown_data_request_callback(&state);

errout_unlock:
  net_unlock();

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
