/****************************************************************************
 * net/usrsock/usrsock_getsockopt.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK) && \
    defined(CONFIG_NET_SOCKOPTS)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t getsockopt_event(FAR struct net_driver_s *dev,
                                 FAR void *pvconn, FAR void *pvpriv,
                                 uint16_t flags)
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
        }
      else
        {
          pstate->valuelen = conn->resp.valuelen;
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
 * Name: do_getsockopt_request
 ****************************************************************************/

static int do_getsockopt_request(FAR struct usrsock_conn_s *conn, int level,
                                 int option, socklen_t value_len)
{
  struct usrsock_request_getsockopt_s req =
  {
  };

  struct iovec bufs[1];

  if (level < INT16_MIN || level > INT16_MAX)
    {
      return -EINVAL;
    }

  if (option < INT16_MIN || option > INT16_MAX)
    {
      return -EINVAL;
    }

  if (value_len > UINT16_MAX)
    {
      value_len = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_GETSOCKOPT;
  req.usockid = conn->usockid;
  req.level = level;
  req.option = option;
  req.max_valuelen = value_len;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Input Parameters:
 *   conn      usrsock socket connection structure
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 ****************************************************************************/

int usrsock_getsockopt(FAR struct usrsock_conn_s *conn,
                       int level, int option,
                       FAR void *value, FAR socklen_t *value_len)
{
  struct usrsock_data_reqstate_s state =
  {
  };

  struct iovec inbufs[1];
  ssize_t ret;

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; getsockopt() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
      goto errout_unlock;
    }

  /* Set up event callback for usrsock. */

  ret = usrsock_setup_data_request_callback(
      conn, &state, getsockopt_event,
      USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
  if (ret < 0)
    {
      nwarn("usrsock_setup_request_callback failed: %d\n", ret);
      goto errout_unlock;
    }

  inbufs[0].iov_base = (FAR void *)value;
  inbufs[0].iov_len = *value_len;

  usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

  /* Request user-space daemon to close socket. */

  ret = do_getsockopt_request(conn, level, option, *value_len);
  if (ret >= 0)
    {
      /* Wait for completion of request. */

      net_lockedwait_uninterruptible(&state.reqstate.recvsem);
      ret = state.reqstate.result;

      DEBUGASSERT(state.valuelen <= *value_len);

      if (ret >= 0)
        {
          /* Store length of data that was written to 'value' buffer. */

          *value_len = state.valuelen;
        }
    }

  usrsock_teardown_datain(conn);
  usrsock_teardown_data_request_callback(&state);

errout_unlock:
  net_unlock();

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK && CONFIG_NET_SOCKOPTS */
