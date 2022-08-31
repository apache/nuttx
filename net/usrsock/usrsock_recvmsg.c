/****************************************************************************
 * net/usrsock/usrsock_recvmsg.c
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

static uint16_t recvfrom_event(FAR struct net_driver_s *dev,
                               FAR void *pvpriv, uint16_t flags)
{
  FAR struct usrsock_data_reqstate_s *pstate = pvpriv;
  FAR struct usrsock_conn_s *conn = pstate->reqstate.conn;

  if (flags & USRSOCK_EVENT_ABORT)
    {
      ninfo("socket aborted.\n");

      pstate->reqstate.result = -ECONNABORTED;
      pstate->valuelen = 0;
      pstate->valuelen_nontrunc = 0;

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

      if (!(flags & USRSOCK_EVENT_RECVFROM_AVAIL) &&
           (pstate->reqstate.result >= 0 ||
            pstate->reqstate.result == -EAGAIN))
        {
          /* After reception of data, mark input not ready. Daemon will
           * send event to restore this flag.
           */

          conn->flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;
        }

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_RECVFROM_AVAIL)
    {
      ninfo("recvfrom avail.\n");

      flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags   = 0;
      pstate->reqstate.cb->priv    = NULL;
      pstate->reqstate.cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("remote closed.\n");

      pstate->reqstate.result = -EPIPE;

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
 * Name: do_recvfrom_request
 ****************************************************************************/

static int do_recvfrom_request(FAR struct usrsock_conn_s *conn,
                               size_t buflen, socklen_t addrlen,
                               int32_t flags)
{
  struct usrsock_request_recvfrom_s req =
  {
  };

  struct iovec bufs[1];

  if (addrlen > UINT16_MAX)
    {
      addrlen = UINT16_MAX;
    }

  if (buflen > UINT32_MAX)
    {
      buflen = UINT32_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_RECVFROM;
  req.usockid = conn->usockid;
  req.flags = flags;
  req.max_addrlen = addrlen;
  req.max_buflen = buflen;

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len = sizeof(req);

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Name: usrsock_recvmsg
 *
 * Description:
 *   recvmsg() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvfrom() will return 0.  Otherwise, on any failure, a negated errno
 *   value is returned.
 *
 ****************************************************************************/

ssize_t usrsock_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                        int flags)
{
  FAR void *buf = msg->msg_iov->iov_base;
  size_t len = msg->msg_iov->iov_len;
  FAR struct sockaddr *from = msg->msg_name;
  FAR socklen_t *fromlen = &msg->msg_namelen;
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state =
  {
  };

  struct iovec inbufs[2];
  socklen_t addrlen = 0;
  socklen_t outaddrlen = 0;
  ssize_t ret;

  DEBUGASSERT(conn);

  if (fromlen)
    {
      if (*fromlen > 0 && from == NULL)
        {
          return -EINVAL;
        }

      addrlen = *fromlen;
    }

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; recvfrom() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
      goto errout_unlock;
    }

  if (conn->type == SOCK_STREAM || conn->type == SOCK_SEQPACKET)
    {
      if (!conn->connected)
        {
          if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
            {
              /* Connecting. */

              ninfo("usockid=%d; socket still connecting.\n",
                    conn->usockid);

              ret = -EAGAIN;
              goto errout_unlock;
            }
          else
            {
              /* Not connected. */

              ninfo("usockid=%d; socket not connected.\n",
                    conn->usockid);

              ret = -ENOTCONN;
              goto errout_unlock;
            }
        }
    }

  if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
    {
      /* Non-blocking connecting. */

      ninfo("usockid=%d; socket still connecting.\n",
            conn->usockid);

      ret = -EAGAIN;
      goto errout_unlock;
    }

  do
    {
      /* Check if remote end has closed connection. */

      if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED &&
          !(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL))
        {
          ninfo("usockid=%d; remote closed (EOF).\n", conn->usockid);

          ret = 0;
          goto errout_unlock;
        }

      /* Check if need to wait for receive data to become available. */

      if (!(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL))
        {
          if (_SS_ISNONBLOCK(conn->sconn.s_flags) ||
              (flags & MSG_DONTWAIT) != 0)
            {
              /* Nothing to receive from daemon side. */

              ret = -EAGAIN;
              goto errout_unlock;
            }

          /* Wait recv to become avail. */

          ret = usrsock_setup_data_request_callback(
              conn, &state, recvfrom_event,
              USRSOCK_EVENT_ABORT | USRSOCK_EVENT_RECVFROM_AVAIL |
              USRSOCK_EVENT_REMOTE_CLOSED);
          if (ret < 0)
            {
              nwarn("usrsock_setup_request_callback failed: %zd\n", ret);
              goto errout_unlock;
            }

          /* Wait for receive-avail (or abort, or timeout, or signal). */

          ret = net_timedwait(&state.reqstate.recvsem,
                              _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  ninfo("recvfrom timedout\n");

                  ret = -EAGAIN;
                }
              else if (ret == -EINTR)
                {
                  ninfo("recvfrom interrupted\n");
                }
              else
                {
                  nerr("net_timedwait errno: %zd\n", ret);
                  DEBUGPANIC();
                }
            }

          usrsock_teardown_data_request_callback(&state);

          /* Did wait timeout or got signal? */

          if (ret != 0)
            {
              goto errout_unlock;
            }

          /* Was socket aborted? */

          if (conn->state == USRSOCK_CONN_STATE_ABORTED)
            {
              ret = -EPIPE;
              goto errout_unlock;
            }

          /* Did remote disconnect? */

          if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED &&
              !(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL))
            {
              ret = 0;
              goto errout_unlock;
            }

          DEBUGASSERT(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL);
        }

      /* Set up event callback for usrsock. */

      ret = usrsock_setup_data_request_callback(
          conn, &state, recvfrom_event,
          USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
      if (ret < 0)
        {
          nwarn("usrsock_setup_request_callback failed: %zd\n", ret);
          goto errout_unlock;
        }

      inbufs[0].iov_base = (FAR void *)from;
      inbufs[0].iov_len = addrlen;
      inbufs[1].iov_base = (FAR void *)buf;
      inbufs[1].iov_len = len;

      usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

      /* MSG_DONTWAIT is only use in usrsock. */

      flags &= ~MSG_DONTWAIT;

      /* Request user-space daemon to close socket. */

      ret = do_recvfrom_request(conn, len, addrlen, flags);
      if (ret >= 0)
        {
          /* Wait for completion of request. */

          net_lockedwait_uninterruptible(&state.reqstate.recvsem);
          ret = state.reqstate.result;

          DEBUGASSERT(ret <= (ssize_t)len);
          DEBUGASSERT(state.valuelen <= addrlen);
          DEBUGASSERT(state.valuelen <= state.valuelen_nontrunc);

          if (ret >= 0)
            {
              /* Store length of 'from' address that was available at
               * daemon-side.
               */

              outaddrlen = state.valuelen_nontrunc;

              /* If the MSG_PEEK flag is enabled, it will only peek
               * from the buffer, so remark the input as ready.
               */

              if (flags & MSG_PEEK)
                {
                  conn->flags |= USRSOCK_EVENT_RECVFROM_AVAIL;
                }
            }
        }

      usrsock_teardown_datain(conn);
      usrsock_teardown_data_request_callback(&state);
    }
  while (ret == -EAGAIN);

errout_unlock:
  net_unlock();

  if (fromlen)
    {
      *fromlen = outaddrlen;
    }

  if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED &&
      ret == -ENOTCONN)
    {
      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
