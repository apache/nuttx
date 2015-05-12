/****************************************************************************
 * fs/vfs/fs_poll.c
 *
 *   Copyright (C) 2008-2009, 2012-2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>

#include <arch/irq.h>

#include "inode/inode.h"

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define poll_semgive(sem) sem_post(sem)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_semtake
 ****************************************************************************/

static int poll_semtake(FAR sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  if (sem_wait(sem) < 0)
    {
      int err = get_errno();

      /* The only case that an error should occur here is if the wait were
       * awakened by a signal.
       */

      DEBUGASSERT(err == EINTR);
      return -err;
    }

  return OK;
}

/****************************************************************************
 * Name: poll_fdsetup
 *
 * Description:
 *   Configure (or unconfigure) one file/socket descriptor for the poll
 *   operation.  If fds and sem are non-null, then the poll is being setup.
 *   if fds and sem are NULL, then the poll is being torn down.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int poll_fdsetup(int fd, FAR struct pollfd *fds, bool setup)
{
  /* Check for a valid file descriptor */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Perform the socket ioctl */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          return net_poll(fd, fds, setup);
        }
      else
#endif
        {
          return -EBADF;
        }
    }

  return file_poll(fd, fds, setup);
}
#endif

/****************************************************************************
 * Name: poll_setup
 *
 * Description:
 *   Setup the poll operation for each descriptor in the list.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static inline int poll_setup(FAR struct pollfd *fds, nfds_t nfds, sem_t *sem)
{
  unsigned int i;
  unsigned int j;
  int ret;

  /* Process each descriptor in the list */

  for (i = 0; i < nfds; i++)
    {
      /* Setup the poll descriptor */

      fds[i].sem     = sem;
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

      if (fds[i].fd >= 0)
        {
          /* Set up the poll on this valid file descriptor */

          ret = poll_fdsetup(fds[i].fd, &fds[i], true);
          if (ret < 0)
            {
              /* Setup failed for fds[i]. We now need to teardown previously
               * setup fds[0 .. (i - 1)] to release allocated resources and
               * to prevent memory corruption by access to freed/released 'fds'
               * and 'sem'.
               */

              for (j = 0; j < i; j++)
                {
                  (void)poll_fdsetup(fds[j].fd, &fds[j], false);
                }

              return ret;
            }
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: poll_teardown
 *
 * Description:
 *   Teardown the poll operation for each descriptor in the list and return
 *   the count of non-zero poll events.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static inline int poll_teardown(FAR struct pollfd *fds, nfds_t nfds, int *count,
                                int ret)
{
  unsigned int i;
  int status;

  /* Process each descriptor in the list */

  *count = 0;
  for (i = 0; i < nfds; i++)
    {
      /* Ignore negative descriptors */

      if (fds[i].fd >= 0)
        {
          /* Teardown the poll */

          status = poll_fdsetup(fds[i].fd, &fds[i], false);
          if (status < 0)
            {
              ret = status;
            }
        }

      /* Check if any events were posted */

      if (fds[i].revents != 0)
        {
          (*count)++;
        }

      /* Un-initialize the poll structure */

      fds[i].sem = NULL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: file_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on file descriptors
 *   to this function.
 *
 * Input Parameters:
 *   fd    - The file descriptor of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
int file_poll(int fd, FAR struct pollfd *fds, bool setup)
{
  FAR struct file *filep;
  FAR struct inode *inode;
  int ret = -ENOSYS;

  /* Get the file pointer corresponding to this file descriptor */

  filep = fs_getfilep(fd);
  if (!filep)
    {
      /* The errno value has already been set */

      return -get_errno();
    }

  /* Is a driver registered? Does it support the poll method?
   * If not, return -ENOSYS
   */

  inode = filep->f_inode;
  if (inode && inode->u.i_ops && inode->u.i_ops->poll)
    {
      /* Yes, then setup the poll */

      ret = (int)inode->u.i_ops->poll(filep, fds, setup);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: poll
 *
 * Description:
 *   poll() waits for one of a set of file descriptors to become ready to
 *   perform I/O.  If none of the events requested (and no error) has
 *   occurred for any of  the  file  descriptors,  then  poll() blocks until
 *   one of the events occurs.
 *
 * Inputs:
 *   fds  - List of structures describing file descriptors to be monitored
 *   nfds - The number of entries in the list
 *   timeout - Specifies an upper limit on the time for which poll() will
 *     block in milliseconds.  A negative value of timeout means an infinite
 *     timeout.
 *
 * Return:
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
  struct timespec abstime;
  irqstate_t flags;
  sem_t sem;
  int count = 0;
  int ret;

  sem_init(&sem, 0, 0);
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
          time_t   sec;
          uint32_t nsec;

          /* Either wait for either a poll event(s), for a signal to occur,
           * or for the specified timeout to elapse with no event.
           *
           * NOTE: If a poll event is pending (i.e., the semaphore has already
           * been incremented), sem_timedwait() will not wait, but will return
           * immediately.
           */

           sec  = timeout / MSEC_PER_SEC;
           nsec = (timeout - MSEC_PER_SEC * sec) * NSEC_PER_MSEC;

           /* Make sure that the following are atomic by disabling interrupts.
            * Interrupts will be re-enabled while we are waiting.
            */

           flags = irqsave();
           (void)clock_gettime(CLOCK_REALTIME, &abstime);

           abstime.tv_sec  += sec;
           abstime.tv_nsec += nsec;
           if (abstime.tv_nsec >= NSEC_PER_SEC)
             {
               abstime.tv_sec++;
               abstime.tv_nsec -= NSEC_PER_SEC;
             }

           ret = sem_timedwait(&sem, &abstime);
           if (ret < 0)
             {
               int err = get_errno();

               if (err == ETIMEDOUT)
                 {
                   /* Return zero (OK) in the event of a timeout */

                   ret = OK;
                 }
               else
                 {
                   /* EINTR is the only other error expected in normal operation */

                   ret = -err;
                 }
             }

           irqrestore(flags);
        }
      else
        {
          /* Wait for the poll event or signal with no timeout */

          ret = poll_semtake(&sem);
        }

      /* Teardown the poll operation and get the count of events.  Zero will be
       * returned in the case of a timeout.
       */

      ret = poll_teardown(fds, nfds, &count, ret);
    }

  sem_destroy(&sem);

  /* Check for errors */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return count;
}

#endif /* CONFIG_DISABLE_POLL */
