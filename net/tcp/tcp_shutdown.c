/****************************************************************************
 * net/tcp/tcp_shutdown.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#ifdef CONFIG_NET_TCP

#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include "utils/utils.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_shutdown_eventhandler
 ****************************************************************************/

static uint32_t tcp_shutdown_eventhandler(FAR struct net_driver_s *dev,
                                          FAR void *pvpriv, uint32_t flags)
{
  FAR struct tcp_conn_s *conn = pvpriv;

  ninfo("flags: %" PRIx32 "\n", flags);

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  /* We don't need the send callback anymore. */

  if (conn->sndcb != NULL)
    {
      conn->sndcb->flags = 0;
      conn->sndcb->event = NULL;

      /* The callback will be freed by tcp_free. */

      conn->sndcb = NULL;
    }
#endif

  dev->d_len = 0;
  if ((conn->shutdown & SHUT_WR) != 0)
    {
      flags |= TCP_TXCLOSE;
    }

  if ((conn->shutdown & SHUT_RD) != 0)
    {
      flags |= TCP_RXCLOSE;
    }

  if (conn->shdcb != NULL)
    {
      tcp_callback_free(conn, conn->shdcb);
      conn->shdcb = NULL;
    }

  return flags;
}

/****************************************************************************
 * Name: tcp_send_fin
 *
 * Description:
 *   Send a FIN for TCP connection
 *
 * Input Parameters:
 *   conn - TCP connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

static inline int tcp_send_fin(FAR struct socket *psock)
{
  FAR struct tcp_conn_s *conn;
  int ret = OK;

  /* Interrupts are disabled here to avoid race conditions */

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  conn_dev_lock(&conn->sconn, conn->dev);
  if ((conn->tcpstateflags == TCP_ESTABLISHED ||
       conn->tcpstateflags == TCP_SYN_SENT ||
       conn->tcpstateflags == TCP_SYN_RCVD ||
       conn->tcpstateflags == TCP_CLOSE_WAIT))
    {
      if ((conn->shdcb = tcp_callback_alloc(conn)) == NULL)
        {
          ret = -EBUSY;
          goto out;
        }

      /* Set up to receive TCP data event callbacks */

      conn->shdcb->flags = TCP_POLL;
      conn->shdcb->event = tcp_shutdown_eventhandler;
      conn->shdcb->priv  = conn; /* reference for event handler to free cb */

      /* Notify the device driver of the availability of TX data */

      tcp_send_txnotify(psock, conn);
    }

out:
  conn_dev_unlock(&conn->sconn, conn->dev);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_shutdown
 *
 * Description:
 *   Gracefully shutdown a TCP connection by sending a FIN
 *
 * Input Parameters:
 *   psock - An instance of the internal socket structure.
 *   how   - Specifies the type of shutdown.
 *
 * Assumptions:
 *   Called from normal user-level logic
 *
 ****************************************************************************/

int tcp_shutdown(FAR struct socket *psock, int how)
{
  FAR struct tcp_conn_s *conn;

  conn = psock->s_conn;
  DEBUGASSERT(conn != NULL);

  if (!(how & SHUT_RDWR))
    {
      return -EOPNOTSUPP;
    }

  conn->shutdown |= how;

  tcp_unlisten(psock->s_conn); /* No longer accepting connections */

  return tcp_send_fin(psock);
}

#endif /* CONFIG_NET_TCP */
