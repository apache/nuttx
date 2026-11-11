/****************************************************************************
 * net/local/local_netpoll.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_event_pollsetup
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
static int local_event_pollsetup(FAR struct local_conn_s *conn,
                                 FAR struct pollfd_s *fds,
                                 bool setup)
{
  int i;

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      nxmutex_lock(&conn->lc_polllock);

      for (i = 0; i < LOCAL_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!conn->lc_event_fds[i])
            {
              /* Bind the poll structure and this slot */

              conn->lc_event_fds[i] = fds;
              fds->priv = &conn->lc_event_fds[i];
              break;
            }
        }

      nxmutex_unlock(&conn->lc_polllock);

      if (i >= LOCAL_NPOLLWAITERS)
        {
          fds->priv = NULL;
          return -EBUSY;
        }

      if (conn->lc_state == LOCAL_STATE_LISTENING &&
          dq_peek(&conn->u.server.lc_waiters) != NULL)
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd_s **slot = (FAR struct pollfd_s **)fds->priv;

      nxmutex_lock(&conn->lc_polllock);

      /* Remove all memory of the poll setup */

      if (slot != NULL)
        {
          *slot = NULL;
          fds->priv = NULL;
        }

      nxmutex_unlock(&conn->lc_polllock);
    }

  return OK;
}

/****************************************************************************
 * Name: local_pollstate
 *
 * Description:
 *   The events of the shutdown state of a socket, as on Linux: when no more
 *   data comes in, read() does not block (POLLIN) and the peer is gone for
 *   reading (POLLRDHUP); when no data goes out either, POLLHUP.
 *
 ****************************************************************************/

static pollevent_t local_pollstate(FAR struct local_conn_s *conn)
{
  pollevent_t eventset = 0;

  if ((conn->lc_shutdown & SHUT_RD) != 0)
    {
      eventset |= POLLIN | POLLRDHUP;
    }

  if ((conn->lc_shutdown & SHUT_RDWR) == SHUT_RDWR)
    {
      eventset |= POLLHUP;
    }

  return eventset;
}

/****************************************************************************
 * Name: local_fifo_poll_cb
 *
 * Description:
 *   An event on a FIFO of the socket.  A FIFO reports POLLHUP when it has
 *   no writer and POLLERR when it has no reader.  For the socket only its
 *   shutdown state counts, which the peer sets before it closes a FIFO.
 *
 ****************************************************************************/

static void local_fifo_poll_cb(FAR struct pollfd_s *fds)
{
  FAR struct local_pollfd_s *lp = fds->arg;
  pollevent_t eventset = fds->revents & (POLLIN | POLLOUT);

  /* The output FIFO has no reader: send() does not block, it fails */

  if (fds == &lp->lp_out && (fds->revents & POLLERR) != 0)
    {
      eventset |= POLLOUT;
    }

  poll_notify(&lp->lp_fds, 1, eventset | local_pollstate(lp->lp_conn));
}

/****************************************************************************
 * Name: local_fifo_pollsetup
 *
 * Description:
 *   Set up the poll of one FIFO of the socket.
 *
 ****************************************************************************/

static int local_fifo_pollsetup(FAR struct local_pollfd_s *lp,
                                FAR struct file *filep,
                                FAR struct pollfd_s *fds,
                                pollevent_t events)
{
  fds->events = events;
  fds->arg    = lp;
  fds->cb     = local_fifo_poll_cb;

  return file_poll(filep, fds, true);
}

/****************************************************************************
 * Name: local_fifo_pollteardown
 ****************************************************************************/

static int local_fifo_pollteardown(FAR struct local_conn_s *conn,
                                   FAR struct local_pollfd_s *lp)
{
  int ret = OK;
  int ret2;

  /* A FIFO that shutdown() closed has no poll left */

  if (lp->lp_in.priv != NULL)
    {
      ret = file_poll(&conn->lc_infile, &lp->lp_in, false);
    }

  if (lp->lp_out.priv != NULL)
    {
      ret2 = file_poll(&conn->lc_outfile, &lp->lp_out, false);
      if (ret2 < 0)
        {
          ret = ret2;
        }
    }

  lp->lp_conn = NULL;
  return ret;
}

/****************************************************************************
 * Name: local_stream_pollsetup
 *
 * Description:
 *   Set up the poll of a connected stream socket: the poll of the FIFOs
 *   for the events that the caller asked for, and the events of the
 *   socket state now.
 *
 ****************************************************************************/

static int local_stream_pollsetup(FAR struct local_conn_s *conn,
                                  FAR struct pollfd_s *fds)
{
  FAR struct local_pollfd_s *lp;
  pollevent_t eventset = 0;
  int ret = OK;
  int i;

  nxmutex_lock(&conn->lc_polllock);

  for (i = 0; i < CONFIG_DEV_PIPE_NPOLLWAITERS; i++)
    {
      if (conn->lc_pfds.stream[i].lp_conn == NULL)
        {
          break;
        }
    }

  if (i >= CONFIG_DEV_PIPE_NPOLLWAITERS)
    {
      nxmutex_unlock(&conn->lc_polllock);
      fds->priv = NULL;
      return -EBUSY;
    }

  lp = &conn->lc_pfds.stream[i];
  lp->lp_conn = conn;
  lp->lp_fds  = fds;
  memset(&lp->lp_in, 0, sizeof(lp->lp_in));
  memset(&lp->lp_out, 0, sizeof(lp->lp_out));

  /* A FIFO that shutdown() closed has nothing to wait for: the state gives
   * POLLIN, and send() does not block (it fails).
   */

  if ((fds->events & (POLLIN | POLLRDHUP)) != 0 &&
      conn->lc_infile.f_inode != NULL)
    {
      ret = local_fifo_pollsetup(lp, &conn->lc_infile, &lp->lp_in, POLLIN);
    }

  if ((fds->events & POLLOUT) != 0)
    {
      if (conn->lc_outfile.f_inode == NULL)
        {
          eventset |= POLLOUT;
        }
      else if (ret >= 0)
        {
          ret = local_fifo_pollsetup(lp, &conn->lc_outfile, &lp->lp_out,
                                     POLLOUT);
        }
    }

  if (ret < 0)
    {
      local_fifo_pollteardown(conn, lp);
      fds->priv = NULL;
    }
  else
    {
      fds->priv = lp;
      poll_notify(&fds, 1, eventset | local_pollstate(conn));
    }

  nxmutex_unlock(&conn->lc_polllock);
  return ret;
}
#endif /* CONFIG_NET_LOCAL_STREAM */

/****************************************************************************
 * Name: local_inout_poll_cb
 ****************************************************************************/

static void local_inout_poll_cb(FAR struct pollfd_s *fds)
{
  FAR struct pollfd_s *originfds = fds->arg;

  poll_notify(&originfds, 1, fds->revents);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_event_pollnotify
 ****************************************************************************/

void local_event_pollnotify(FAR struct local_conn_s *conn,
                            pollevent_t eventset)
{
#ifdef CONFIG_NET_LOCAL_STREAM
  nxmutex_lock(&conn->lc_polllock);
  poll_notify(conn->lc_event_fds, LOCAL_NPOLLWAITERS, eventset);
  nxmutex_unlock(&conn->lc_polllock);
#endif
}

/****************************************************************************
 * Name: local_shutdown_pollnotify
 *
 * Description:
 *   shutdown() changed the state of a stream socket and is about to close
 *   one of its FIFOs.  Stop the polls of that FIFO and tell all polls of
 *   the socket the new state.
 *
 * Input Parameters:
 *   conn  - The socket
 *   filep - lc_infile or lc_outfile, which shutdown() closes next
 *
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
void local_shutdown_pollnotify(FAR struct local_conn_s *conn,
                               FAR struct file *filep)
{
  FAR struct local_pollfd_s *lp;
  FAR struct pollfd_s *fds;
  int i;

  nxmutex_lock(&conn->lc_polllock);

  for (i = 0; i < CONFIG_DEV_PIPE_NPOLLWAITERS; i++)
    {
      lp = &conn->lc_pfds.stream[i];
      if (lp->lp_conn == NULL)
        {
          continue;
        }

      fds = filep == &conn->lc_infile ? &lp->lp_in : &lp->lp_out;
      if (fds->priv != NULL)
        {
          file_poll(filep, fds, false);
        }

      poll_notify(&lp->lp_fds, 1, local_pollstate(conn) |
                  (filep == &conn->lc_outfile ? POLLOUT : 0));
    }

  nxmutex_unlock(&conn->lc_polllock);
}
#endif

/****************************************************************************
 * Name: local_pollsetup
 *
 * Description:
 *   Setup to monitor events on one Unix domain socket
 *
 * Input Parameters:
 *   psock - The Unix domain socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int local_pollsetup(FAR struct socket *psock, FAR struct pollfd_s *fds)
{
  FAR struct local_conn_s *conn = psock->s_conn;
  int ret = OK;

#ifdef CONFIG_NET_LOCAL_DGRAM
  if (conn->lc_proto == SOCK_DGRAM &&
      conn->lc_state != LOCAL_STATE_CONNECTED)
    {
      /* A datagram send opens its FIFO on demand and has no send buffer to
       * wait for, so it is always possible to send.  Report POLLOUT
       * immediately rather than polling lc_infile, which is read-only and
       * would never become writable.
       */

      if ((fds->events & POLLOUT) != 0)
        {
          fds->revents |= POLLOUT;
          poll_notify(&fds, 1, POLLOUT);
        }

      /* Only an bound socket has a receiver FIFO to watch for input. */

      if ((fds->events & POLLIN) != 0 && conn->lc_infile.f_inode != NULL)
        {
          return file_poll(&conn->lc_infile, fds, true);
        }

      return OK;
    }
#endif

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      return local_event_pollsetup(conn, fds, true);
    }

  if (conn->lc_proto == SOCK_STREAM)
    {
      if (conn->lc_state == LOCAL_STATE_CONNECTED)
        {
          return local_stream_pollsetup(conn, fds);
        }

      /* Not connected, or the connection was lost */

      fds->priv = NULL;
      if (conn->lc_state == LOCAL_STATE_DISCONNECTED ||
          (fds->events & (POLLIN | POLLOUT)) != 0)
        {
          poll_notify(&fds, 1, POLLERR);
        }

      return OK;
    }
#endif

  /* A socket of a datagram socket pair: POLLIN from the FIFO that it
   * reads, POLLOUT from the FIFO that it writes.
   */

  if (conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      fds->priv = NULL;
      goto pollerr;
    }

  switch (fds->events & (POLLIN | POLLOUT))
    {
      case (POLLIN | POLLOUT):
        {
          FAR struct pollfd_s *shadowfds;

          /* Poll wants to check state for both input and output. */

          if (conn->lc_infile.f_inode == NULL ||
              conn->lc_outfile.f_inode == NULL)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          /* Find shadow pollfds. */

          nxmutex_lock(&conn->lc_polllock);

          shadowfds = conn->lc_pfds.pair;
          while (shadowfds->fd != 0)
            {
              shadowfds += 2;
              if (shadowfds >= &conn->lc_pfds.pair[2*LOCAL_NPOLLWAITERS])
                {
                  nxmutex_unlock(&conn->lc_polllock);
                  return -ENOMEM;
                }
            }

          shadowfds[0]         = *fds;
          shadowfds[0].fd      = 1; /* Does not matter */
          shadowfds[0].cb      = local_inout_poll_cb;
          shadowfds[0].arg     = fds;
          shadowfds[0].events &= ~POLLOUT;

          shadowfds[1]         = *fds;
          shadowfds[1].fd      = 0; /* Does not matter */
          shadowfds[1].cb      = local_inout_poll_cb;
          shadowfds[1].arg     = fds;
          shadowfds[1].events &= ~POLLIN;

          nxmutex_unlock(&conn->lc_polllock);

          /* Setup poll for both shadow pollfds. */

          ret = file_poll(&conn->lc_infile, &shadowfds[0], true);
          if (ret >= 0)
            {
              ret = file_poll(&conn->lc_outfile, &shadowfds[1], true);
              if (ret < 0)
                {
                  file_poll(&conn->lc_infile, &shadowfds[0], false);
                }
            }

          if (ret < 0)
            {
              shadowfds[0].fd = 0;
              fds->priv = NULL;
              goto pollerr;
            }
          else
            {
              fds->priv = shadowfds;
            }
        }
        break;

      case POLLIN:
        {
          /* Poll wants to check state for input only. */

          if (conn->lc_infile.f_inode == NULL)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          ret = file_poll(&conn->lc_infile, fds, true);
        }
        break;

      case POLLOUT:
        {
          /* Poll wants to check state for output only. */

          if (conn->lc_outfile.f_inode == NULL)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          ret = file_poll(&conn->lc_outfile, fds, true);
        }
        break;

      default:
        break;
    }

  return ret;

pollerr:
  poll_notify(&fds, 1, POLLERR);
  return OK;
}

/****************************************************************************
 * Name: local_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on a Unix domain socket
 *
 * Input Parameters:
 *   psock - The Unix domain socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int local_pollteardown(FAR struct socket *psock, FAR struct pollfd_s *fds)
{
  FAR struct local_conn_s *conn = psock->s_conn;
  int ret = OK;

#ifdef CONFIG_NET_LOCAL_DGRAM
  if (conn->lc_proto == SOCK_DGRAM &&
      conn->lc_state != LOCAL_STATE_CONNECTED)
    {
      /* Only a registered poll (fds->priv set by setup) needs teardown. */

      if (fds->priv != NULL)
        {
          return file_poll(&conn->lc_infile, fds, false);
        }

      return OK;
    }
#endif

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      return local_event_pollsetup(conn, fds, false);
    }

  if (conn->lc_proto == SOCK_STREAM)
    {
      /* Only a poll that the setup registered needs a teardown */

      if (fds->priv != NULL)
        {
          nxmutex_lock(&conn->lc_polllock);
          ret = local_fifo_pollteardown(conn, fds->priv);
          fds->priv = NULL;
          nxmutex_unlock(&conn->lc_polllock);
        }

      return ret;
    }
#endif

  if (conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      return OK;
    }

  switch (fds->events & (POLLIN | POLLOUT))
    {
      case (POLLIN | POLLOUT):
        {
          FAR struct pollfd_s *shadowfds = fds->priv;
          int ret2;

          if (shadowfds == NULL)
            {
              return OK;
            }

          /* Teardown for both shadow pollfds. */

          ret = file_poll(&conn->lc_infile, &shadowfds[0], false);
          ret2 = file_poll(&conn->lc_outfile, &shadowfds[1], false);
          if (ret2 < 0)
            {
              ret = ret2;
            }

          fds->revents |= shadowfds[0].revents | shadowfds[1].revents;
          fds->priv = NULL;
          shadowfds[0].fd = 0;
        }
        break;

      case POLLIN:
        {
          if (fds->priv == NULL)
            {
              return OK;
            }

          ret = file_poll(&conn->lc_infile, fds, false);
        }
        break;

      case POLLOUT:
        {
          if (fds->priv == NULL)
            {
              return OK;
            }

          ret = file_poll(&conn->lc_outfile, fds, false);
        }
        break;

      default:
        break;
    }

  return ret;
}
