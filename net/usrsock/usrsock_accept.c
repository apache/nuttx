/****************************************************************************
 * net/usrsock/usrsock_accept.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/usrsock.h>

#include "usrsock/usrsock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t accept_event(FAR struct net_driver_s *dev,
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

      pstate->reqstate.cb->flags = 0;
      pstate->reqstate.cb->priv  = NULL;
      pstate->reqstate.cb->event = NULL;

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

      if (pstate->reqstate.result >= 0 ||
          pstate->reqstate.result == -EAGAIN)
        {
          /* After reception of connection, mark input not ready. Daemon will
           * send event to restore this flag.
           */

          conn->flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;
        }

      /* Stop further callbacks */

      pstate->reqstate.cb->flags = 0;
      pstate->reqstate.cb->priv  = NULL;
      pstate->reqstate.cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_REMOTE_CLOSED)
    {
      ninfo("remote closed.\n");

      pstate->reqstate.result = -EPIPE;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags = 0;
      pstate->reqstate.cb->priv  = NULL;
      pstate->reqstate.cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }
  else if (flags & USRSOCK_EVENT_RECVFROM_AVAIL)
    {
      ninfo("accept avail.\n");

      flags &= ~USRSOCK_EVENT_RECVFROM_AVAIL;

      /* Stop further callbacks */

      pstate->reqstate.cb->flags = 0;
      pstate->reqstate.cb->priv  = NULL;
      pstate->reqstate.cb->event = NULL;

      /* Wake up the waiting thread */

      nxsem_post(&pstate->reqstate.recvsem);
    }

  return flags;
}

/****************************************************************************
 * Name: do_accept_request
 ****************************************************************************/

static int do_accept_request(FAR struct usrsock_conn_s *conn,
                             socklen_t addrlen)
{
  struct usrsock_request_accept_s req =
  {
  };

  struct iovec bufs[1];

  if (addrlen > UINT16_MAX)
    {
      addrlen = UINT16_MAX;
    }

  /* Prepare request for daemon to read. */

  req.head.reqid = USRSOCK_REQUEST_ACCEPT;
  req.usockid = conn->usockid;
  req.max_addrlen = addrlen;

  bufs[0].iov_base = &req;
  bufs[0].iov_len = sizeof(req);

  return usrsock_do_request(conn, bufs, ARRAY_SIZE(bufs));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usrsock_accept
 *
 * Description:
 *   The usrsock_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'psock', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'psock' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an usrsock_accept.
 *
 *   The 'psock' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, usrsock_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, usrsock_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr',
 *            Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a description of the appropriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int usrsock_accept(FAR struct socket *psock, FAR struct sockaddr *addr,
                   FAR socklen_t *addrlen, FAR struct socket *newsock)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  struct usrsock_data_reqstate_s state =
  {
  };

  FAR struct usrsock_conn_s *newconn;
  struct iovec inbufs[2];
  socklen_t inaddrlen = 0;
  socklen_t outaddrlen = 0;
  int ret;

  DEBUGASSERT(conn);

  if (addrlen)
    {
      if (*addrlen > 0 && addr == NULL)
        {
          return -EINVAL;
        }

      inaddrlen = *addrlen;
    }

  net_lock();

  if (conn->state == USRSOCK_CONN_STATE_UNINITIALIZED ||
      conn->state == USRSOCK_CONN_STATE_ABORTED)
    {
      /* Invalid state or closed by daemon. */

      ninfo("usockid=%d; accept() with uninitialized usrsock.\n",
            conn->usockid);

      ret = (conn->state == USRSOCK_CONN_STATE_ABORTED) ? -EPIPE :
            -ECONNRESET;
      goto errout_unlock;
    }

  if (conn->state == USRSOCK_CONN_STATE_CONNECTING)
    {
      /* Connecting. */

      ninfo("usockid=%d; socket still connecting.\n",
            conn->usockid);

      ret = -EAGAIN;
      goto errout_unlock;
    }

  if (!conn->connected)
    {
      /* Not connected. */

      ninfo("usockid=%d; socket not connected.\n",
            conn->usockid);

      ret = -ENOTCONN;
      goto errout_unlock;
    }

  /* Allocate the usrsock socket connection structure and save in the new
   * socket instance.
   */

  newconn = usrsock_alloc();
  if (!newconn)
    {
      /* Failed to reserve a connection structure */

      ret = -ENOMEM;
      goto errout_unlock;
    }

  do
    {
      /* Check if remote end has closed connection. */

      if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
        {
          ninfo("usockid=%d; remote closed (EOF).\n", conn->usockid);

          ret = -EPIPE;
          goto errout_free_conn;
        }

      /* Check if need to wait for connection to become available. */

      if (!(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL))
        {
          if (_SS_ISNONBLOCK(conn->sconn.s_flags))
            {
              /* Nothing to receive from daemon side. */

              ret = -EAGAIN;
              goto errout_free_conn;
            }

          /* Wait accept to become avail. */

          ret = usrsock_setup_data_request_callback(
              conn, &state, accept_event,
              USRSOCK_EVENT_ABORT | USRSOCK_EVENT_RECVFROM_AVAIL |
              USRSOCK_EVENT_REMOTE_CLOSED);
          if (ret < 0)
            {
              nwarn("usrsock_setup_request_callback failed: %d\n", ret);
              goto errout_free_conn;
            }

          /* Wait for receive-avail (or abort, or timeout, or signal). */

          ret = net_sem_timedwait(&state.reqstate.recvsem,
                              _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
          usrsock_teardown_data_request_callback(&state);
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  ninfo("accept timedout\n");

                  ret = -EAGAIN;
                }
              else if (ret == -EINTR)
                {
                  ninfo("accept interrupted\n");
                }
              else
                {
                  nerr("net_sem_timedwait errno: %d\n", ret);
                  DEBUGPANIC();
                }

              goto errout_free_conn;
            }

          /* Was socket aborted? */

          if (conn->state == USRSOCK_CONN_STATE_ABORTED)
            {
              ret = -EPIPE;
              goto errout_free_conn;
            }

          /* Did remote disconnect? */

          if (conn->flags & USRSOCK_EVENT_REMOTE_CLOSED)
            {
              ret = -EPIPE;
              goto errout_free_conn;
            }

          DEBUGASSERT(conn->flags & USRSOCK_EVENT_RECVFROM_AVAIL);
        }

      /* Set up event callback for usrsock. */

      ret = usrsock_setup_data_request_callback(
          conn, &state, accept_event,
          USRSOCK_EVENT_ABORT | USRSOCK_EVENT_REQ_COMPLETE);
      if (ret < 0)
        {
          nwarn("usrsock_setup_request_callback failed: %d\n", ret);
          goto errout_free_conn;
        }

      inbufs[0].iov_base = addr;
      inbufs[0].iov_len = inaddrlen;
      inbufs[1].iov_base = &newconn->usockid;
      inbufs[1].iov_len = sizeof(newconn->usockid);

      usrsock_setup_datain(conn, inbufs, ARRAY_SIZE(inbufs));

      /* We might start getting events for this socket right after
       * returning to daemon, so setup 'newconn' already here.
       */

      newconn->state = USRSOCK_CONN_STATE_READY;

      /* Request user-space daemon to accept socket. */

      ret = do_accept_request(conn, inaddrlen);
      if (ret >= 0)
        {
          /* Wait for completion of request. */

          net_sem_wait_uninterruptible(&state.reqstate.recvsem);
          ret = state.reqstate.result;

          DEBUGASSERT(state.valuelen <= inaddrlen);
          DEBUGASSERT(state.valuelen <= state.valuelen_nontrunc);

          if (ret >= 0)
            {
              newconn->connected = true;
              newconn->crefs     = 1;

              newsock->s_type    = psock->s_type;
              newsock->s_domain  = psock->s_domain;
              newsock->s_conn    = newconn;
              newsock->s_sockif  = psock->s_sockif;

              /* Store length of 'from' address that was available at
               * daemon-side.
               */

              outaddrlen = state.valuelen_nontrunc;
              ret = OK;
            }
        }

      usrsock_teardown_datain(conn);
      usrsock_teardown_data_request_callback(&state);
    }
  while (ret == -EAGAIN);

  if (ret >= 0)
    {
      goto errout_unlock;
    }

errout_free_conn:
  usrsock_free(newconn);

errout_unlock:
  net_unlock();

  if (addrlen)
    {
      *addrlen = outaddrlen;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
