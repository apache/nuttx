/****************************************************************************
 * net/usrsock/usrsock_getsockname.c
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

#include <arch/irq.h>

#include <sys/socket.h>
#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t getsockname_event(FAR struct net_driver_s *dev,
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
 * Name: do_getsockopt_request
 ****************************************************************************/

static int do_getsockname_request(FAR struct usrsock_conn_s *conn,
                                  socklen_t addrlen)
{
  struct usrsock_request_getsockname_s req =
  {
  };

  struct iovec bufs[1];

  if (addrlen > UINT16_MAX)
    {
      addrlen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_GETSOCKNAME;
  req.usockid = conn->usockid;
  req.max_addrlen = addrlen;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsock_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 ****************************************************************************/

int usrsock_getsockname(FAR struct socket *psock,
                        FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state =
  {
  };

  struct iovec inbufs[1];
  socklen_t outaddrlen = 0;
  int ret;

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; getsockname() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
      goto errout_unlock;
    }

  /* Set up event callback for usrsock. */

  ret = usrsock_setup_data_request_callback(
      conn, &state, getsockname_event,
      USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
  if (ret < 0)
    {
      nwarn("usrsock_setup_request_callback failed: %d\n", ret);
      goto errout_unlock;
    }

  inbufs[0].iov_base = (FAR void *)addr;
  inbufs[0].iov_len = *addrlen;

  usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

  /* Request user-space daemon to handle request. */

  ret = do_getsockname_request(conn, *addrlen);
  if (ret >= 0)
    {
      /* Wait for completion of request. */

      net_lockedwait_uninterruptible(&state.reqstate.recvsem);
      ret = state.reqstate.result;

      DEBUGASSERT(state.valuelen <= *addrlen);
      DEBUGASSERT(state.valuelen <= state.valuelen_nontrunc);

      if (ret >= 0)
        {
          /* Store length of data that was written to 'value' buffer. */

          outaddrlen = state.valuelen_nontrunc;
        }
    }

  usrsock_teardown_datain(conn);
  usrsock_teardown_data_request_callback(&state);

errout_unlock:
  net_unlock();

  *addrlen = outaddrlen;
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
