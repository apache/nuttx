/****************************************************************************
 * net/usrsock/usrsock_event.c
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

  if (events & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      /* After reception of remote close event, clear input/output flags. */

      conn->flags &= ~(USRSOCK_EVENT_SENDTO_READY |
                       USRSOCK_EVENT_RECVFROM_AVAIL);

      conn->flags |= USRSOCK_EVENT_REMOTE_CLOSED;
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

  /* Send events to callbacks */

  devif_conn_event(NULL, conn, events, conn->list);
  net_unlock();

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
