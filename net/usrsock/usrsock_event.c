/****************************************************************************
 * net/usrsock/usrsock_event.c
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

#include "devif/devif.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_event
 *
 * Description:
 *   Handler for received connection events
 *
 ****************************************************************************/

int usrsock_event(FAR struct usrsock_conn_s *conn, uint16_t events)
{
  ninfo("events: %04X\n", events);

  if (!events)
    {
      return OK;
    }

  net_lock();

  /* Generic state updates. */

  if (events & USRSOCK_EVENT_REQ_COMPLETE)
    {
      if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
        {
          conn->state = USRSOCK_CONN_STATE_READY;
          events |= USRSOCK_EVENT_CONNECT_READY;

          if (conn->resp.result == 0)
            {
              conn->connected = true;
            }
        }
    }

  if (events & USRSOCK_EVENT_ABORT)
    {
      conn->state = USRSOCK_CONN_STATE_ABORTED;
    }

  if ((conn->state == USRSOCK_CONN_STATE_READY ||
       conn->state == USRSOCK_CONN_STATE_CONNECTING) &&
      !(conn->flags & USRSOCK_EVENT_REMOTE_CLOSED))
    {
      if (events & USRSOCK_EVENT_SENDTO_READY)
        {
          conn->flags |= USRSOCK_EVENT_SENDTO_READY;
        }

      if (events & USRSOCK_EVENT_RECVFROM_AVAIL)
        {
          conn->flags |= USRSOCK_EVENT_RECVFROM_AVAIL;
        }
    }

  if (events & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      /* After reception of remote close event, clear input flags. */

      conn->flags &= ~USRSOCK_EVENT_SENDTO_READY;

      conn->flags |= USRSOCK_EVENT_REMOTE_CLOSED;
    }

  /* Send events to callbacks */

  devif_conn_event(NULL, events, conn->sconn.list);
  net_unlock();

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
