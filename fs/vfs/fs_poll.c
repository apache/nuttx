/****************************************************************************
 * fs/vfs/fs_poll.c
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

#include <stdbool.h>
#include <poll.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include <arch/irq.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_fdsetup
 *
 * Description:
 *   Configure (or unconfigure) one file/socket descriptor for the poll
 *   operation.  If fds and sem are non-null, then the poll is being setup.
 *   if fds and sem are NULL, then the poll is being torn down.
 *
 ****************************************************************************/

static int poll_fdsetup(int fd, FAR struct pollfd *fds, bool setup)
{
  FAR struct file *filep;
  int ret;

  /* Get the file pointer corresponding to this file descriptor */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(filep != NULL);

  /* Let file_poll() do the rest */

  return file_poll(filep, fds, setup);
}

/****************************************************************************
 * Name: poll_default_cb
 *
 * Description:
 *   The default poll callback function, this function do the final step of
 *   poll notification.
 *
 * Input Parameters:
 *   fds - The fds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void poll_default_cb(FAR struct pollfd *fds)
{
  int semcount = 0;
  FAR sem_t *pollsem;

  if (fds->arg != NULL)
    {
      pollsem = (FAR sem_t *)fds->arg;
      nxsem_get_value(pollsem, &semcount);
      if (semcount < 1)
        {
          nxsem_post(pollsem);
        }
    }
}

/****************************************************************************
 * Name: poll_setup
 *
 * Description:
 *   Setup the poll operation for each descriptor in the list.
 *
 ****************************************************************************/

static inline int poll_setup(FAR struct pollfd *fds, nfds_t nfds,
                             FAR sem_t *sem)
{
  unsigned int i;
  unsigned int j;
  int ret = OK;

  /* Process each descriptor in the list */

  for (i = 0; i < nfds; i++)
    {
      /* Setup the poll descriptor
       *
       * REVISIT: In a multi-threaded environment, one use case might be to
       * share a single, array of struct pollfd in poll() calls on different
       * threads.  That use case is not supportable here because the non-
       * standard internal fields get reset here on each call to poll()
       * on each thread.
       */

      fds[i].arg     = sem;
      fds[i].cb      = poll_default_cb;
      fds[i].revents = 0;
      fds[i].priv    = NULL;

      /* Check for invalid descriptors. "If the value of fd is less than 0,
       * events shall be ignored, and revents shall be set to 0 in that entry
       * on return from poll()."
       *
       * NOTE:  There is a potential problem here.  If there is only one fd
       * and if it is negative, then poll will hang.  From my reading of the
       * spec, that appears to be the correct behavior.
       */

      switch (fds[i].events & POLLMASK)
        {
        case POLLFD:
          if (fds[i].fd >= 0)
            {
              ret = poll_fdsetup(fds[i].fd, &fds[i], true);
            }
          break;

        case POLLFILE:
          if (fds[i].ptr != NULL)
            {
              ret = file_poll(fds[i].ptr, &fds[i], true);
            }
          break;

#ifdef CONFIG_NET
        case POLLSOCK:
          if (fds[i].ptr != NULL)
            {
              ret = psock_poll(fds[i].ptr, &fds[i], true);
            }
          break;
#endif

        default:
          ret = -EINVAL;
          break;
        }

      if (ret < 0)
        {
          /* Setup failed for fds[i]. We now need to teardown previously
           * setup fds[0 .. (i - 1)] to release allocated resources and
           * to prevent memory corruption by access to freed/released 'fds'
           * and 'sem'.
           */

          for (j = 0; j < i; j++)
            {
              switch (fds[j].events & POLLMASK)
                {
                case POLLFD:
                  poll_fdsetup(fds[j].fd, &fds[j], false);
                  break;

                case POLLFILE:
                  file_poll(fds[j].ptr, &fds[j], false);
                  break;

#ifdef CONFIG_NET
                case POLLSOCK:
                  psock_poll(fds[j].ptr, &fds[j], false);
                  break;
#endif

                default:
                  break;
                }
            }

          /* Indicate an error on the file descriptor */

          fds[i].revents |= POLLERR;
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: poll_teardown
 *
 * Description:
 *   Teardown the poll operation for each descriptor in the list and return
 *   the count of non-zero poll events.
 *
 ****************************************************************************/

static inline int poll_teardown(FAR struct pollfd *fds, nfds_t nfds,
                                FAR int *count, int ret)
{
  unsigned int i;
  int status = OK;

  /* Process each descriptor in the list */

  *count = 0;
  for (i = 0; i < nfds; i++)
    {
      switch (fds[i].events & POLLMASK)
        {
        case POLLFD:
          if (fds[i].fd >= 0)
            {
              status = poll_fdsetup(fds[i].fd, &fds[i], false);
            }
          break;

        case POLLFILE:
          if (fds[i].ptr != NULL)
            {
              status = file_poll(fds[i].ptr, &fds[i], false);
            }
          break;

#ifdef CONFIG_NET
        case POLLSOCK:
            if (fds[i].ptr != NULL)
            {
              status = psock_poll(fds[i].ptr, &fds[i], false);
            }
          break;
#endif

        default:
          status = -EINVAL;
          break;
        }

      if (status < 0)
        {
          ret = status;
        }

      /* Check if any events were posted */

      if (fds[i].revents != 0)
        {
          (*count)++;
        }

      /* Un-initialize the poll structure */

      fds[i].arg = NULL;
      fds[i].cb  = NULL;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_notify
 *
 * Description:
 *   Notify the poll, this function should be called by drivers to notify
 *   the caller the poll is ready.
 *
 * Input Parameters:
 *   afds     - The fds array
 *   nfds     - Number of fds array
 *   eventset - List of events to check for activity
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void poll_notify(FAR struct pollfd **afds, int nfds, pollevent_t eventset)
{
  int i;
  FAR struct pollfd *fds;

  DEBUGASSERT(afds != NULL && nfds >= 1);

  for (i = 0; i < nfds && eventset; i++)
    {
      fds = afds[i];
      if (fds != NULL)
        {
          /* The error event must be set in fds->revents */

          fds->revents |= eventset & (fds->events | POLLERR | POLLHUP);
          if ((fds->revents & (POLLERR | POLLHUP)) != 0)
            {
              /* Error or Hung up, clear POLLOUT event */

              fds->revents &= ~POLLOUT;
            }

          if (fds->revents != 0 && fds->cb != NULL)
            {
              finfo("Report events: %08" PRIx32 "\n", fds->revents);
              fds->cb(fds);
            }
        }
    }
}

/****************************************************************************
 * Name: file_poll
 *
 * Description:
 *   Low-level poll operation based on struct file.  This is used both to (1)
 *   support detached file, and also (2) by poll_fdsetup() to perform all
 *   normal operations on file descriptors.
 *
 * Input Parameters:
 *   file     File structure instance
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int file_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  int ret = -ENOSYS;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  if (inode != NULL)
    {
      /* Is a driver registered? Does it support the poll method?
       * If not, return -ENOSYS
       */

      if ((INODE_IS_DRIVER(inode) || INODE_IS_MQUEUE(inode) ||
          INODE_IS_SOCKET(inode)) &&
          inode->u.i_ops != NULL && inode->u.i_ops->poll != NULL)
        {
          /* Yes, it does... Setup the poll */

          ret = (int)inode->u.i_ops->poll(filep, fds, setup);
        }

      /* Regular files (and block devices) are always readable and
       * writable. Open Group: "Regular files shall always poll TRUE for
       * reading and writing."
       */

      if (INODE_IS_MOUNTPT(inode) || INODE_IS_BLOCK(inode) ||
          INODE_IS_MTD(inode))
        {
          if (setup)
            {
              poll_notify(&fds, 1, POLLIN | POLLOUT);
            }

          ret = OK;
        }
    }
  else
    {
      poll_notify(&fds, 1, POLLERR | POLLHUP);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: nx_poll
 *
 * Description:
 *   nx_poll() is similar to the standard 'poll' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_poll() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_poll(FAR struct pollfd *fds, unsigned int nfds, int timeout)
{
  sem_t sem;
  int count = 0;
  int ret2;
  int ret;

  DEBUGASSERT(nfds == 0 || fds != NULL);

  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&sem, 0, 0);
  nxsem_set_protocol(&sem, SEM_PRIO_NONE);

  ret = poll_setup(fds, nfds, &sem);
  if (ret >= 0)
    {
      if (timeout == 0)
        {
          /* Poll returns immediately whether we have a poll event or not. */

          ret = OK;
        }
      else if (timeout > 0)
        {
          clock_t ticks;

          /* "Implementations may place limitations on the granularity of
           * timeout intervals. If the requested timeout interval requires
           * a finer granularity than the implementation supports, the
           * actual timeout interval will be rounded up to the next
           * supported value." -- opengroup.org
           *
           * Round timeout up to next full tick.
           */

#if (MSEC_PER_TICK * USEC_PER_MSEC) != USEC_PER_TICK && \
    defined(CONFIG_HAVE_LONG_LONG)
          ticks = (((unsigned long long)timeout * USEC_PER_MSEC) +
                   (USEC_PER_TICK - 1)) /
                  USEC_PER_TICK;
#else
          ticks = ((unsigned int)timeout + (MSEC_PER_TICK - 1)) /
                  MSEC_PER_TICK;
#endif

          /* Either wait for either a poll event(s), for a signal to occur,
           * or for the specified timeout to elapse with no event.
           *
           * NOTE: If a poll event is pending (i.e., the semaphore has
           * already been incremented), nxsem_tickwait() will not wait, but
           * will return immediately.
           */

          ret = nxsem_tickwait(&sem, ticks);
          if (ret < 0)
            {
              if (ret == -ETIMEDOUT)
                {
                  /* Return zero (OK) in the event of a timeout */

                  ret = OK;
                }

              /* EINTR is the only other error expected in normal operation */
            }
        }
      else
        {
          /* Wait for the poll event or signal with no timeout */

          ret = nxsem_wait(&sem);
        }

      /* Teardown the poll operation and get the count of events.  Zero will
       * be returned in the case of a timeout.
       *
       * Preserve ret, if negative, since it holds the result of the wait.
       */

      ret2 = poll_teardown(fds, nfds, &count, ret);
      if (ret2 < 0 && ret >= 0)
        {
          ret = ret2;
        }
    }

  nxsem_destroy(&sem);
  return ret < 0 ? ret : count;
}

/****************************************************************************
 * Name: poll
 *
 * Description:
 *   poll() waits for one of a set of file descriptors to become ready to
 *   perform I/O.  If none of the events requested (and no error) has
 *   occurred for any of  the  file  descriptors,  then  poll() blocks until
 *   one of the events occurs.
 *
 * Input Parameters:
 *   fds  - List of structures describing file descriptors to be monitored
 *   nfds - The number of entries in the list
 *   timeout - Specifies an upper limit on the time for which poll() will
 *     block in milliseconds.  A negative value of timeout means an infinite
 *     timeout.
 *
 * Returned Value:
 *   On success, the number of structures that have non-zero revents fields.
 *   A value of 0 indicates that the call timed out and no file descriptors
 *   were ready.  On error, -1 is returned, and errno is set appropriately:
 *
 *   EBADF  - An invalid file descriptor was given in one of the sets.
 *   EFAULT - The fds address is invalid
 *   EINTR  - A signal occurred before any requested event.
 *   EINVAL - The nfds value exceeds a system limit.
 *   ENOMEM - There was no space to allocate internal data structures.
 *   ENOSYS - One or more of the drivers supporting the file descriptor
 *     does not support the poll method.
 *
 ****************************************************************************/

int poll(FAR struct pollfd *fds, nfds_t nfds, int timeout)
{
  int ret;

  /* poll() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_poll() do all of the work */

  ret = nx_poll(fds, nfds, timeout);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
