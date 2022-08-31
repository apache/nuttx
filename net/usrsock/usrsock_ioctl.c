/****************************************************************************
 * net/usrsock/usrsock_ioctl.c
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
#include <debug.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>
#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
#  include <nuttx/wireless/wireless.h>
#endif
#include "socket/socket.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t ioctl_event(FAR struct net_driver_s *dev,
                            FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_data_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pstate->reqstate.conn;

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
  struct usrsock_request_ioctl_s req =
  {
  };

  struct iovec bufs[3] =
  {
  };

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

#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
  if (WL_IS80211POINTERCMD(cmd))
    {
      FAR struct iwreq *wlreq = arg;
      bufs[2].iov_base = wlreq->u.data.pointer;
      bufs[2].iov_len = wlreq->u.data.length;
    }
#endif

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_ioctl
 *
 * Description:
 *   This function performs network device specific operations.
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *   arglen   The length of 'arg'
 *
 ****************************************************************************/

int usrsock_ioctl(FAR struct socket *psock, int cmd, FAR void *arg,
                  size_t arglen)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state =
  {
  };

  struct iovec inbufs[2] =
  {
  };

  int ret;

  /* Bypass FIONBIO to socket level,
   * since the usrsock server always put the socket in nonblocking mode.
   */

  if (cmd == FIONBIO)
    {
      return -ENOTTY;
    }

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; ioctl() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
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

#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
  if (WL_IS80211POINTERCMD(cmd))
    {
      FAR struct iwreq *wlreq = arg;
      inbufs[1].iov_base = wlreq->u.data.pointer;
      inbufs[1].iov_len = wlreq->u.data.length;
    }
#endif

  usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

  /* Request user-space daemon to handle ioctl. */

  ret = do_ioctl_request(conn, cmd, arg, arglen);
  if (ret >= 0)
    {
      /* Wait for completion of request. */

      net_lockedwait_uninterruptible(&state.reqstate.recvsem);
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
