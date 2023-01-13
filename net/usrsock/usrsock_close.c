/****************************************************************************
 * net/usrsock/usrsock_close.c
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

static uint16_t close_event(FAR struct net_driver_s *dev,
                            FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pstate->conn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      conn->state = USRSOCK_CONN_STATE_ABORTED;
      pstate->result = -ECONNABORTED;

      /* Stop further callbacks */

      pstate->cb->flags = 0;
      pstate->cb->priv  = NULL;
      pstate->cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_REQ_COMPLETE)
    {
      ninfo("request completed.\n");

      pstate->result = conn->resp.result;

      /* Stop further callbacks */

      pstate->cb->flags = 0;
      pstate->cb->priv  = NULL;
      pstate->cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_close_request
 ****************************************************************************/

static int do_close_request(FAR struct usrsock_conn_s *conn)
{
  struct usrsock_request_close_s req =
  {
  };

  struct iovec bufs[1];

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_CLOSE;
  req.usockid = conn->usockid;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsock_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_close
 *
 * Description:
 *
 ****************************************************************************/

int usrsock_close(FAR struct usrsock_conn_s *conn)
{
  struct usrsock_reqstate_s state =
  {
  };

  int ret;

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Already closed? */

      ninfo("usockid=%d; already closed.\n", conn->usockid);

      ret = OK;
      goto close_out;
    }

  /* Set up event callback for usrsock. */

  ret = usrsock_setup_request_callback(conn, &state, close_event,
                                       USRSOCK_EVENT_ABORT |
                                       USRSOCK_EVENT_REQ_COMPLETE);
  if (ret < 0)
    {
      nwarn("usrsock_setup_request_callback failed: %d\n", ret);
      goto errout;
    }

  /* Request user-space daemon to close socket. */

  ret = do_close_request(conn);
  if (ret < 0)
    {
      ret = OK; /* Error? return OK for close. */
    }
  else
    {
      /* Wait for completion of request. */

      net_sem_wait_uninterruptible(&state.recvsem);

      ret = state.result;
      if (ret < 0)
        {
          /* TODO: Error handling for close? Mark closed anyway? There is not
           * much we can do if this happens.
           */

          ninfo("user-space daemon reported error %d for usockid=%d\n",
                state.result, conn->usockid);

          ret = OK;
        }
    }

  usrsock_teardown_request_callback(&state);

close_out:
  conn->state = USRSOCK_CONN_STATE_UNINITIALIZED;
  conn->usockid = -1;

errout:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
