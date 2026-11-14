/****************************************************************************
 * fs/vfs/fs_signalfd.c
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
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/poll.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/signalfd.h>

#include <sys/signalfd.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the internal state of the driver */

struct signalfd_priv_s
{
  sigset_t      sigmask; /* The set of signals caller wishes */
  mutex_t       mutex;   /* Enforces device exclusive access */
  uint8_t       crefs;   /* References counts on signalfd (max: 255) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int signalfd_file_open(FAR struct file *filep);
static int signalfd_file_close(FAR struct file *filep);
static ssize_t signalfd_file_read(FAR struct file *filep,
                                  FAR char *buffer, size_t len);
static int signalfd_file_poll(FAR struct file *filep,
                              FAR struct pollfd_s *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_signalfd_fileops =
{
  signalfd_file_open,   /* open */
  signalfd_file_close,  /* close */
  signalfd_file_read,   /* read */
  NULL,                 /* write */
  NULL,                 /* seek */
  NULL,                 /* ioctl */
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  signalfd_file_poll    /* poll */
};

static struct inode g_signalfd_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_DRIVER, /* i_flags */
  {
    &g_signalfd_fileops   /* u */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int signalfd_file_open(FAR struct file *filep)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;
  int ret;

  nxmutex_lock(&dev->mutex);
  if (dev->crefs >= 255)
    {
      ret = -EMFILE;
    }
  else
    {
      dev->crefs += 1;
      ret = OK;
    }

  nxmutex_unlock(&dev->mutex);
  return ret;
}

static int signalfd_file_close(FAR struct file *filep)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;

  nxmutex_lock(&dev->mutex);
  if (dev->crefs > 1)
    {
      dev->crefs--;
      nxmutex_unlock(&dev->mutex);
      return OK;
    }

  nxmutex_unlock(&dev->mutex);
  nxmutex_destroy(&dev->mutex);
  fs_heap_free(dev);

  return OK;
}

static ssize_t signalfd_file_read(FAR struct file *filep,
                                  FAR char *buffer, size_t len)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;
  struct signalfd_siginfo siginfo;
  struct siginfo info;
  sigset_t pendmask;
  size_t nread = 0;
  ssize_t ret;
  int count;

  if (buffer == NULL)
    {
      return -EFAULT;
    }

  if (len < sizeof(struct signalfd_siginfo))
    {
      return -EINVAL;
    }

  count = len / sizeof(struct signalfd_siginfo);

  pendmask = nxsig_pendingset(NULL);
  sigandset(&pendmask, &pendmask, &dev->sigmask);
  if (sigisemptyset(&pendmask))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }
      else
        {
          pendmask = dev->sigmask;
        }
    }

  do
    {
      ret = nxsig_timedwait(&pendmask, &info, NULL);
      if (ret < 0)
        {
          goto errout;
        }

      memset(&siginfo, 0, sizeof(siginfo));
      siginfo.ssi_signo  = info.si_signo;
      siginfo.ssi_errno  = info.si_errno;
      siginfo.ssi_code   = info.si_code;
#ifdef CONFIG_SCHED_HAVE_PARENT
      siginfo.ssi_pid    = info.si_pid;
      siginfo.ssi_uid    = info.si_uid;
      siginfo.ssi_status = info.si_status;
#endif
      siginfo.ssi_int    = info.si_value.sival_int;
      siginfo.ssi_ptr    = (uint64_t)(uintptr_t)info.si_value.sival_ptr;

      memcpy(buffer + nread, &siginfo, sizeof(siginfo));
      nread += sizeof(siginfo);

      pendmask = nxsig_pendingset(NULL);
      sigandset(&pendmask, &pendmask, &dev->sigmask);
    }
  while (--count != 0 && !sigisemptyset(&pendmask));

errout:
  return nread > 0 ? (ssize_t)nread : ret;
}

static int signalfd_file_poll(FAR struct file *filep,
                              FAR struct pollfd_s *fds, bool setup)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;
  FAR struct task_group_s *group = nxsched_self()->group;
  FAR struct pollfd_s **cand = NULL;
  irqstate_t flags;
  sigset_t mask;
  int ret = 0;
  int i;

  /* The waiter array is allocated lazily so a group that never polls a
   * signalfd carries only a NULL pointer.  Allocate the candidate outside
   * the critical section (allocation must not run under
   * enter_critical_section); it is installed under the lock and freed here
   * if another concurrent poll setup won the race.  tg_sigfds only ever
   * transitions NULL->set (freed at group teardown), so the unlocked
   * pre-check is a safe hint.
   */

  if (setup && group->tg_sigfds == NULL)
    {
      cand = kmm_zalloc(CONFIG_SIGNAL_FD_NPOLLWAITERS * sizeof(*cand));
      if (cand == NULL)
        {
          return -ENOMEM;
        }
    }

  /* The poll waiters are anchored on the task group and are notified from
   * the signal dispatch path, which may run in interrupt context.  Use
   * enter_critical_section() (switch-aware, so poll_notify()'s nxsem_post()
   * may switch out safely) rather than the device mutex, which the dispatch
   * path could not take.
   */

  flags = enter_critical_section();
  if (!setup)
    {
      /* This is a request to tear down the poll.  Locate our slot in the
       * group anchor by pointer and clear it.
       */

      if (group->tg_sigfds != NULL)
        {
          for (i = 0; i < CONFIG_SIGNAL_FD_NPOLLWAITERS; i++)
            {
              if (group->tg_sigfds[i] == fds)
                {
                  group->tg_sigfds[i] = NULL;
                  break;
                }
            }
        }

      fds->priv = NULL;
      goto out;
    }

  /* Install the lazily-allocated array if it is still missing. */

  if (group->tg_sigfds == NULL)
    {
      group->tg_sigfds = cand;
      cand             = NULL;
    }

  /* This is a request to set up the poll. Find an available slot in the
   * group anchor and bind the poll structure to it.  The device pointer is
   * stashed in fds->priv so signalfd_notify() can filter by this fd's mask;
   * this is safe because an occupied slot implies poll_setup() holds a file
   * reference, so the device cannot be closed/freed while bound.
   */

  for (i = 0; i < CONFIG_SIGNAL_FD_NPOLLWAITERS; i++)
    {
      if (!group->tg_sigfds[i])
        {
          group->tg_sigfds[i] = fds;
          fds->priv           = dev;
          break;
        }
    }

  if (i >= CONFIG_SIGNAL_FD_NPOLLWAITERS)
    {
      ret = -EBUSY;
      goto out;
    }

  /* Notify the POLLIN event if a subscribed signal is already pending */

  mask = nxsig_pendingset(NULL);
  sigandset(&mask, &mask, &dev->sigmask);
  if (!sigisemptyset(&mask))
    {
      poll_notify(&fds, 1, POLLIN);
    }

out:
  leave_critical_section(flags);

  /* Free the candidate array if another poll setup installed one first. */

  if (cand != NULL)
    {
      kmm_free(cand);
    }

  return ret;
}

/****************************************************************************
 * Name: signalfd_notify
 *
 * Description:
 *   Notify the signalfd poll waiters of a task group that signo has been
 *   delivered.  Called from the signal dispatch path, which may run in
 *   interrupt context; enter_critical_section() serializes against
 *   signalfd_file_poll() setup/teardown and is switch-aware, so the
 *   nxsem_post() inside poll_notify() may switch out safely.
 *
 *   Only the fds whose device mask contains signo are notified, so that
 *   several signalfd fds sharing one poll loop do not get spurious POLLIN
 *   for signals they did not subscribe to.  An occupied slot implies
 *   poll_setup() holds a file reference on that fd, so fds->priv (the
 *   device) is guaranteed live for the duration of this walk.
 *
 ****************************************************************************/

void signalfd_notify(FAR struct task_group_s *group, int signo)
{
  irqstate_t flags;
  int i;

  /* A group that never set up a signalfd poll has no waiter array. */

  if (group == NULL || group->tg_sigfds == NULL)
    {
      return;
    }

  flags = enter_critical_section();
  for (i = 0; i < CONFIG_SIGNAL_FD_NPOLLWAITERS; i++)
    {
      FAR struct pollfd_s *fds = group->tg_sigfds[i];

      if (fds != NULL)
        {
          FAR struct signalfd_priv_s *dev = fds->priv;

          if (sigismember(&dev->sigmask, signo) > 0)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: signalfd
 *
 * Description:
 *   signalfd() creates a file descriptor that can be used to accept signals
 *   targeted at the caller.
 *
 *   The mask argument specifies the set of signals that the caller wishes
 *   to accept via the file descriptor. This argument is a signal set whose
 *   contents can be initialized using the macros described in sigsetops.
 *   Normally, the set of signals to be received via the file descriptor
 *   should be blocked using sigprocmask, to prevent the signals being
 *   handled according to their default dispositions. It is not possible
 *   to receive SIGKILL or SIGSTOP signals via a signalfd file descriptor.
 *   these signals are silently ignored if specified in mask.
 *
 *   If the fd argument is -1, then the call creates a new file descriptor
 *   and associates the signal set specified in mask with that file
 *   descriptor. If fd is not -1, then it must specify a valid existing
 *   signalfd file descriptor, and mask is used to replace the signal
 *   set associated with that file descriptor.
 *
 *   The following values may be bitwise ORed in flags to change the
 *   behavior of signalfd():
 *
 *   SFD_NONBLOCK  Set the O_NONBLOCK file status flag on the open file
 *                 description referred to by the new file descriptor.
 *                 Using this flag saves extra calls to fcntl to achieve
 *                 the same result.
 *
 *   SFD_CLOEXEC   Set the close-on-exec (FD_CLOEXEC) flag on the new file
 *                 descriptor. See the description of the O_CLOEXEC flag
 *                 in open for reasons why this may be useful.
 *
 * Returned Value:
 *   On success, signalfd() returns a signalfd file descriptor; this is
 *   either a new file descriptor (if fd was -1), or fd if fd was a valid
 *   signalfd file descriptor. On error, -1 is returned and errno is set
 *   to indicate the error.
 *
 ****************************************************************************/

int signalfd(int fd, FAR const sigset_t *mask, int flags)
{
  FAR struct signalfd_priv_s *dev;
  FAR struct file *filep = NULL;
  int ret = EINVAL;

  if (mask == NULL)
    {
      ret = EFAULT;
      goto errout;
    }

  if (flags & ~(SFD_CLOEXEC | SFD_NONBLOCK))
    {
      goto errout;
    }

  if (fd == -1)
    {
      dev = fs_heap_zalloc(sizeof(*dev));
      if (dev == NULL)
        {
          ret = ENOMEM;
          goto errout;
        }

      nxmutex_init(&dev->mutex);

      fd = file_allocate_from_inode(&g_signalfd_inode, O_RDONLY | flags,
                                    0, dev, 0);
      if (fd < 0)
        {
          ret = -fd;
          goto errout_with_dev;
        }

      dev->crefs++;
    }
  else
    {
      if (file_get(fd, &filep) < 0)
        {
          ret = EBADF;
          goto errout;
        }

      if (filep->f_inode == NULL ||
          filep->f_inode->u.i_ops != &g_signalfd_fileops)
        {
          file_put(filep);
          goto errout;
        }

      dev = filep->f_priv;
    }

  /* Update the subscribed signal set.  signalfd_notify() reads dev->sigmask
   * live when filtering, so a replaced mask takes effect immediately with no
   * per-signal action to (un)register.
   */

  dev->sigmask = *mask;
  nxsig_delset(&dev->sigmask, SIGKILL);
  nxsig_delset(&dev->sigmask, SIGSTOP);

  if (filep != NULL)
    {
      file_put(filep);
    }

  return fd;

errout_with_dev:
  nxmutex_destroy(&dev->mutex);
  fs_heap_free(dev);

errout:
  set_errno(ret);
  return ERROR;
}
