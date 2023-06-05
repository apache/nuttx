/****************************************************************************
 * fs/vfs/fs_signalfd.c
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
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <debug.h>

#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <sys/signalfd.h>

#include "inode/inode.h"

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
  FAR struct pollfd *fds[CONFIG_SIGNAL_FD_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int signalfd_file_open(FAR struct file *filep);
static int signalfd_file_close(FAR struct file *filep);
static ssize_t signalfd_file_read(FAR struct file *filep,
                                  FAR char *buffer, size_t len);
static int signalfd_file_poll(FAR struct file *filep,
                              FAR struct pollfd *fds, bool setup);

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

static void signalfd_action(int signo, FAR siginfo_t *info,
                            FAR void *ucontext)
{
  FAR struct signalfd_priv_s *dev = info->si_user;

  if (sigismember(&dev->sigmask, signo) > 0)
    {
      poll_notify(dev->fds, CONFIG_SIGNAL_FD_NPOLLWAITERS, POLLIN);
    }
}

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
  int signo;

  nxmutex_lock(&dev->mutex);
  if (dev->crefs > 1)
    {
      dev->crefs--;
      nxmutex_unlock(&dev->mutex);
      return OK;
    }

  for (signo = MIN_SIGNO; signo <= MAX_SIGNO; signo++)
    {
      if (nxsig_ismember(&dev->sigmask, signo))
        {
          signal(signo, SIG_DFL);
        }
    }

  nxmutex_unlock(&dev->mutex);
  nxmutex_destroy(&dev->mutex);
  kmm_free(dev);

  return OK;
}

static ssize_t signalfd_file_read(FAR struct file *filep,
                                  FAR char *buffer, size_t len)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;
  FAR struct signalfd_siginfo *siginfo;
  struct siginfo info;
  sigset_t pendmask;
  ssize_t ret;
  int count;

  count = len / sizeof(struct signalfd_siginfo);
  if (buffer == NULL || count == 0)
    {
      return -EINVAL;
    }

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

  siginfo = (FAR struct signalfd_siginfo *)buffer;
  do
    {
      ret = nxsig_waitinfo(&pendmask, &info);
      if (ret < 0)
        {
          goto errout;
        }

      memset(siginfo, 0, sizeof(*siginfo));
      siginfo->ssi_signo  = info.si_signo;
      siginfo->ssi_errno  = info.si_errno;
      siginfo->ssi_code   = info.si_code;
#ifdef CONFIG_SCHED_HAVE_PARENT
      siginfo->ssi_pid    = info.si_pid;
      siginfo->ssi_status = info.si_status;
#endif
      siginfo->ssi_int    = info.si_value.sival_int;
      siginfo->ssi_ptr    = (uint64_t)(uintptr_t)info.si_value.sival_ptr;
      siginfo++;
      pendmask = nxsig_pendingset(NULL);
      sigandset(&pendmask, &pendmask, &dev->sigmask);
    }
  while (--count != 0 && !sigisemptyset(&pendmask));

errout:
  len = (FAR char *)siginfo - buffer;
  return len > 0 ? len : ret;
}

static int signalfd_file_poll(FAR struct file *filep,
                              FAR struct pollfd *fds, bool setup)
{
  FAR struct signalfd_priv_s *dev = filep->f_priv;
  sigset_t mask;
  int ret = 0;
  int i;

  nxmutex_lock(&dev->mutex);
  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
      goto out;
    }

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_SIGNAL_FD_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!dev->fds[i])
        {
          /* Bind the poll structure and this slot */

          dev->fds[i] = fds;
          fds->priv   = &dev->fds[i];
          break;
        }
    }

  if (i >= CONFIG_SIGNAL_FD_NPOLLWAITERS)
    {
      ret = -EBUSY;
      goto out;
    }

  /* Notify the POLLIN event if the counter is not zero */

  mask = nxsig_pendingset(NULL);
  sigandset(&mask, &mask, &dev->sigmask);
  if (!sigisemptyset(&mask))
    {
      poll_notify(dev->fds, CONFIG_SIGNAL_FD_NPOLLWAITERS, POLLIN);
    }

out:
  nxmutex_unlock(&dev->mutex);
  return ret;
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
  struct sigaction act;
  int ret = EINVAL;
  int signo;

  if (flags & ~(SFD_CLOEXEC | SFD_NONBLOCK))
    {
      goto errout;
    }

  if (fd == -1)
    {
      dev = kmm_zalloc(sizeof(*dev));
      if (dev == NULL)
        {
          ret = ENOMEM;
          goto errout;
        }

      nxmutex_init(&dev->mutex);

      fd = file_allocate(&g_signalfd_inode, O_RDOK | flags,
                         0, dev, 0, true);
      if (fd < 0)
        {
          ret = -fd;
          goto errout_with_dev;
        }
    }
  else
    {
      FAR struct file *filep;

      if (fs_getfilep(fd, &filep) < 0)
        {
          ret = EBADF;
          goto errout;
        }

      if (filep->f_inode->u.i_ops != &g_signalfd_fileops)
        {
          goto errout;
        }

      dev = filep->f_priv;
      for (signo = MIN_SIGNO; signo <= MAX_SIGNO; signo++)
        {
          if (nxsig_ismember(&dev->sigmask, signo))
            {
              signal(signo, SIG_DFL);
            }
        }
    }

  dev->sigmask = *mask;
  nxsig_delset(&dev->sigmask, SIGKILL);
  nxsig_delset(&dev->sigmask, SIGSTOP);
  act.sa_sigaction = signalfd_action;
  act.sa_flags = SA_KERNELHAND | SA_SIGINFO;
  act.sa_user = dev;
  for (signo = MIN_SIGNO; signo <= MAX_SIGNO; signo++)
    {
      if (nxsig_ismember(&dev->sigmask, signo))
        {
          nxsig_action(signo, &act, NULL, false);
        }
    }

  return fd;

errout_with_dev:
  nxmutex_destroy(&dev->mutex);
  kmm_free(dev);

errout:
  set_errno(ret);
  return ERROR;
}
