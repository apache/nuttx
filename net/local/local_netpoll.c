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

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/fs/fs.h>

#include "socket/socket.h"
#include "local/local.h"

#ifdef HAVE_LOCAL_POLL

/****************************************************************************
 * Function: local_accept_pollsetup
 ****************************************************************************/

#ifdef CONFIG_NET_LOCAL_STREAM
static int local_accept_pollsetup(FAR struct local_conn_s *conn,
                                  FAR struct pollfd *fds,
                                  bool setup)
{
  net_lock_t state;
  pollevent_t eventset;
  int ret = OK;
  int i;

  state = net_lock();
  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < LOCAL_ACCEPT_NPOLLWAITERS; i++)
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

      if (i >= LOCAL_ACCEPT_NPOLLWAITERS)
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
  net_unlock(state);
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

  for (i = 0; i < LOCAL_ACCEPT_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = conn->lc_accept_fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & eventset);
          if (fds->revents != 0)
            {
              ndbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
#endif
}

/****************************************************************************
 * Function: local_pollsetup
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

          if (conn->lc_infd < 0 || conn->lc_outfd < 0)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          /* Allocate shadow pollfds. */

          shadowfds = kmm_zalloc(2 * sizeof(struct pollfd));
          if (!shadowfds)
            {
              return -ENOMEM;
            }

          shadowfds[0].fd = conn->lc_infd;
          shadowfds[0].sem = fds->sem;
          shadowfds[0].events = fds->events & ~POLLOUT;

          shadowfds[1].fd = conn->lc_outfd;
          shadowfds[1].sem = fds->sem;
          shadowfds[1].events = fds->events & ~POLLIN;

          /* Setup poll for both shadow pollfds. */

          ret = file_poll(conn->lc_infd, &shadowfds[0], true);
          if (ret >= 0)
            {
              ret = file_poll(conn->lc_outfd, &shadowfds[1], true);
              if (ret < 0)
                {
                  (void)file_poll(conn->lc_infd, &shadowfds[0], false);
                }
            }

          if (ret < 0)
            {
              kmm_free(shadowfds);
              fds->priv = NULL;
              goto pollerr;
            }
          else
            {
              fds->priv = shadowfds;
              ret = OK;
            }
        }
        break;

      case POLLIN:
        {
          /* Poll wants to check state for input only. */

          if (conn->lc_infd < 0)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          ret = file_poll(conn->lc_infd, fds, true);
        }
        break;

      case POLLOUT:
        {
          /* Poll wants to check state for output only. */

          if (conn->lc_outfd < 0)
            {
              fds->priv = NULL;
              goto pollerr;
            }

          ret = file_poll(conn->lc_outfd, fds, true);
        }
        break;

      default:
        ret = OK;
        break;
    }
#endif

  return ret;

pollerr:
  fds->revents |= POLLERR;
  sem_post(fds->sem);
  return OK;
}

/****************************************************************************
 * Function: local_pollteardown
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
  int status = OK;
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

          if (shadowfds == NULL)
            {
              return OK;
            }

          DEBUGASSERT(shadowfds[0].fd == conn->lc_infd);
          DEBUGASSERT(shadowfds[1].fd == conn->lc_outfd);

          /* Teardown for both shadow pollfds. */

          ret = file_poll(conn->lc_infd, &shadowfds[0], false);
          if (ret < 0)
            {
              status = ret;
            }

          ret = file_poll(conn->lc_outfd, &shadowfds[1], false);
          if (ret < 0)
            {
              status = ret;
            }

          fds->revents |= shadowfds[0].revents | shadowfds[1].revents;
          fds->priv = NULL;
          kmm_free(shadowfds);
        }
        break;

      case POLLIN:
        {
          if (fds->priv == NULL)
            {
              return OK;
            }

          status = file_poll(conn->lc_infd, fds, false);
        }
        break;

      case POLLOUT:
        {
          if (fds->priv == NULL)
            {
              return OK;
            }

          status = file_poll(conn->lc_outfd, fds, false);
        }
        break;

      default:
        break;
    }
#endif

  return status;
}

#endif /* HAVE_LOCAL_POLL */
