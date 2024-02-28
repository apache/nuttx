/****************************************************************************
 * net/local/local_netpoll.c
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
#include <debug.h>
#include <poll.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>

#include "socket/socket.h"
#include "local/local.h"

/****************************************************************************
 * Name: local_event_pollsetup
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
static int local_event_pollsetup(FAR struct local_conn_s *conn,
                                 FAR struct pollfd *fds,
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

      struct pollfd **slot = (struct pollfd **)fds->priv;

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
 * Name: local_inout_poll_cb
 ****************************************************************************/

static void local_inout_poll_cb(FAR struct pollfd *fds)
{
  FAR struct pollfd *originfds = fds->arg;

  poll_notify(&originfds, 1, fds->revents);
}
#endif

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

int local_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct local_conn_s *conn = psock->s_conn;
  int ret = OK;

  if (conn->lc_proto == SOCK_DGRAM)
    {
      return -ENOSYS;
    }

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      return local_event_pollsetup(conn, fds, true);
    }

  if (conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      fds->priv = NULL;
      goto pollerr;
    }

  switch (fds->events & (POLLIN | POLLOUT))
    {
      case (POLLIN | POLLOUT):
        {
          FAR struct pollfd *shadowfds;

          /* Poll wants to check state for both input and output. */

          if (conn->lc_infile.f_inode == NULL ||
              conn->lc_outfile.f_inode == NULL)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          /* Find shadow pollfds. */

          nxmutex_lock(&conn->lc_polllock);

          shadowfds = conn->lc_inout_fds;
          while (shadowfds->fd != 0)
            {
              shadowfds += 2;
              if (shadowfds >= &conn->lc_inout_fds[2*LOCAL_NPOLLWAITERS])
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
#endif

  return ret;

#ifdef CONFIG_NET_LOCAL_STREAM
pollerr:
  poll_notify(&fds, 1, POLLERR);
  return OK;
#endif
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

int local_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds)
{
  FAR struct local_conn_s *conn = psock->s_conn;
  int ret = OK;

  if (conn->lc_proto == SOCK_DGRAM)
    {
      return -ENOSYS;
    }

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING)
    {
      return local_event_pollsetup(conn, fds, false);
    }

  if (conn->lc_state == LOCAL_STATE_DISCONNECTED)
    {
      return OK;
    }

  switch (fds->events & (POLLIN | POLLOUT))
    {
      case (POLLIN | POLLOUT):
        {
          FAR struct pollfd *shadowfds = fds->priv;
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
#endif

  return ret;
}
