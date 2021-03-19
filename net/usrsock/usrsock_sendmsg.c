/****************************************************************************
 * net/usrsock/usrsock_sendmsg.c
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

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t sendto_event(FAR struct net_driver_s *dev, FAR void *pvconn,
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

      if (pstate->result >= 0 || pstate->result == -EAGAIN)
        {
          /* After reception of data, mark input not ready. Daemon will
           * send event to restore this flag.
           */

          conn->flags &= ~USRSOCK_EVENT_SENDTO_READY;
        }

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("remote closed.\n");

      pstate->result = -EPIPE;

      /* Stop further callbacks */

      pstate->cb->flags   = 0;
      pstate->cb->priv    = NULL;
      pstate->cb->event   = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->recvsem);
    }
  else if (flags & USRSOCK_EVENT_SENDTO_READY)
    {
      ninfo("sendto ready.\n");

      /* Do not let other waiters to claim new data. */

      flags &= ~USRSOCK_EVENT_SENDTO_READY;

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
 * Name: do_sendto_request
 ****************************************************************************/

static int do_sendto_request(FAR struct usrsock_conn_s *conn,
                             FAR struct msghdr *msg, int flags)
{
  struct usrsock_request_sendto_s req =
  {
  };

  struct iovec bufs[2 + msg->msg_iovlen];
  int i;

  if (msg->msg_namelen > UINT16_MAX)
    {
      msg->msg_namelen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_SENDTO;
  req.usockid = conn->usockid;
  req.flags = flags;
  req.addrlen = msg->msg_namelen;

  for (i = 0; i < msg->msg_iovlen; i++)
    {
      req.buflen += msg->msg_iov[i].iov_len;
    }

  if (req.buflen > UINT16_MAX)
    {
      req.buflen = UINT16_MAX;
    }

  bufs[0].iov_base = (FAR void *)&req;
  bufs[0].iov_len  = sizeof(req);
  bufs[1].iov_base = msg->msg_name;
  bufs[1].iov_len  = msg->msg_namelen;

  memcpy(&bufs[2], msg->msg_iov, sizeof(struct iovec) * msg->msg_iovlen);

  return usrsockdev_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_sendmsg
 *
 * Description:
 *   If sendmsg() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters 'msg_name' and 'msg_namelen' are ignored (and the
 *   error EISCONN may be returned when they are not NULL and 0), and the
 *   error ENOTCONN is returned when the socket was not actually connected.
 *
 * Input Parameters:
 *   psock    A reference to the socket structure of the socket
 *   msg      Message to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On any failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t usrsock_sendmsg(FAR struct socket *psock,
                        FAR struct msghdr *msg, int flags)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_reqstate_s state =
  {
  };

  ssize_t ret;

  DEBUGASSERT(conn);

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; sendto() with uninitialized usrsock.\n",
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

              ret = -ENOTCONN;
              goto errout_unlock;
            }
        }

      if (msg->msg_name || msg->msg_namelen)
        {
          /* Address provided for connection-mode socket */

          ret = -EISCONN;
          goto errout_unlock;
        }
    }

  if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
    {
      /* Non-blocking connecting. */

      ninfo("usockid=%d; socket still connecting.\n", conn->usockid);

      ret = -EAGAIN;
      goto errout_unlock;
    }

  do
    {
      /* Check if remote end has closed connection. */

      if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
        {
          ninfo("usockid=%d; remote closed.\n", conn->usockid);

          ret = -EPIPE;
          goto errout_unlock;
        }

      /* Check if need to wait for send to become ready. */

      if (!(conn->flags & USRSOCK_EVENT_SENDTO_READY))
        {
          if (_SS_ISNONBLOCK(psock->s_flags) || (flags & MSG_DONTWAIT) != 0)
            {
              /* Send busy at daemon side. */

              ret = -EAGAIN;
              goto errout_unlock;
            }

          /* Wait send to become ready. */

          ret = usrsock_setup_request_callback(conn, &state, sendto_event,
                                               USRSOCK_EVENT_ABORT |
                                               USRSOCK_EVENT_SENDTO_READY |
                                               USRSOCK_EVENT_REMOTE_CLOSED);
          if (ret < 0)
            {
              nwarn("usrsock_setup_request_callback failed: %d\n", ret);
              goto errout_unlock;
            }

          /* Wait for send-ready (or abort, or timeout, or signal). */

          ret = net_timedwait(&state.recvsem,
                              _SO_TIMEOUT(psock->s_sndtimeo));
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  ninfo("sendto timedout\n");

                  ret = -EAGAIN;
                }
              else if (ret == -EINTR)
                {
                  ninfo("sendto interrupted\n");
                }
              else
                {
                  nerr("net_timedwait errno: %d\n", ret);
                  DEBUGASSERT(false);
                }
            }

          usrsock_teardown_request_callback(&state);

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

          if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
            {
              ret = -EPIPE;
              goto errout_unlock;
            }

          DEBUGASSERT(conn->flags & USRSOCK_EVENT_SENDTO_READY);
        }

      /* Set up event callback for usrsock. */

      ret = usrsock_setup_request_callback(conn, &state, sendto_event,
                                           USRSOCK_EVENT_ABORT |
                                           USRSOCK_EVENT_REQ_COMPLETE);
      if (ret < 0)
        {
          nwarn("usrsock_setup_request_callback failed: %d\n", ret);
          goto errout_unlock;
        }

      /* MSG_DONTWAIT is only use in usrsock. */

       flags &= ~MSG_DONTWAIT;

      /* Request user-space daemon to close socket. */

      ret = do_sendto_request(conn, msg, flags);
      if (ret >= 0)
        {
          /* Wait for completion of request. */

          net_lockedwait_uninterruptible(&state.recvsem);
          ret = state.result;
        }

      usrsock_teardown_request_callback(&state);
    }
  while (ret == -EAGAIN);

errout_unlock:
  net_unlock();
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
