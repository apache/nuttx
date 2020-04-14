/****************************************************************************
 * net/local/local_netpoll.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>

#include "socket/socket.h"
#include "local/local.h"

#ifdef HAVE_LOCAL_POLL

/****************************************************************************
 * Name: local_accept_pollsetup
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
static int local_accept_pollsetup(FAR struct local_conn_s *conn,
                                  FAR struct pollfd *fds,
                                  bool setup)
{
  pollevent_t eventset;
  int ret = OK;
  int i;

  net_lock();
  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < LOCAL_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!conn->lc_accept_fds[i])
            {
              /* Bind the poll structure and this slot */

              conn->lc_accept_fds[i] = fds;
              fds->priv = &conn->lc_accept_fds[i];
              break;
            }
        }

      if (i >= LOCAL_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto errout;
        }

      eventset = 0;
      if (dq_peek(&conn->u.server.lc_waiters) != NULL)
        {
          eventset |= POLLIN;
        }

      if (eventset)
        {
          local_accept_pollnotify(conn, eventset);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      if (!slot)
        {
          ret = -EIO;
          goto errout;
        }

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  net_unlock();
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: local_accept_pollnotify
 ****************************************************************************/

void local_accept_pollnotify(FAR struct local_conn_s *conn,
                             pollevent_t eventset)
{
#ifdef CONFIG_NET_LOCAL_STREAM
  int i;

  for (i = 0; i < LOCAL_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = conn->lc_accept_fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              ninfo("Report events: %02x\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
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
  FAR struct local_conn_s *conn;
  int ret = -ENOSYS;

  conn = (FAR struct local_conn_s *)psock->s_conn;

  if (conn->lc_proto == SOCK_DGRAM)
    {
      return ret;
    }

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING &&
      conn->lc_type  == LOCAL_TYPE_PATHNAME)
    {
      return local_accept_pollsetup(conn, fds, true);
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

          net_lock();

          shadowfds = conn->lc_inout_fds;
          while (shadowfds->fd != 0)
            {
              shadowfds += 2;
              if (shadowfds >= &conn->lc_inout_fds[2*LOCAL_NPOLLWAITERS])
                {
                  net_unlock();
                  return -ENOMEM;
                }
            }

          shadowfds[0].fd     = 1; /* Does not matter */
          shadowfds[0].sem    = fds->sem;
          shadowfds[0].events = fds->events & ~POLLOUT;

          shadowfds[1].fd     = 0; /* Does not matter */
          shadowfds[1].sem    = fds->sem;
          shadowfds[1].events = fds->events & ~POLLIN;

          net_unlock();

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
        ret = OK;
        break;
    }
#endif

  return ret;

#ifdef CONFIG_NET_LOCAL_STREAM
pollerr:
  fds->revents |= POLLERR;
  nxsem_post(fds->sem);
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
  FAR struct local_conn_s *conn;
  int ret = OK;

  conn = (FAR struct local_conn_s *)psock->s_conn;

  if (conn->lc_proto == SOCK_DGRAM)
    {
      return -ENOSYS;
    }

#ifdef CONFIG_NET_LOCAL_STREAM
  if (conn->lc_state == LOCAL_STATE_LISTENING &&
      conn->lc_type  == LOCAL_TYPE_PATHNAME)
    {
      return local_accept_pollsetup(conn, fds, false);
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

#endif /* HAVE_LOCAL_POLL */
