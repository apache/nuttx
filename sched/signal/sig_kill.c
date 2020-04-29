/****************************************************************************
 * sched/signal/sig_kill.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2013, 2015, 2017 Gregory Nutt. All
 *     rights reserved.
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

#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_kill
 *
 * Description:
 *   The nxsig_kill() system call can be used to send any signal to any task
 *   group.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the POSIX standard kill() function but does not modify the application
 *   errno variable.
 *
 *   Limitation: Sending of signals to 'process groups' is not
 *   supported in NuttX
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX nxsig_kill
 *     specification encodes process group information as zero and
 *     negative pid values.  Only positive, non-zero values of pid are
 *     supported by this implementation.
 *   signo - The signal number to send.  If signo is zero, no signal is
 *     sent, but all error checking is performed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EINVAL An invalid signal was specified.
 *    EPERM  The process does not have permission to send the
 *           signal to any of the target processes.
 *    ESRCH  The pid or process group does not exist.
 *    ENOSYS Do not support sending signals to process groups.
 *
 ****************************************************************************/

int nxsig_kill(pid_t pid, int signo)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
#endif
  siginfo_t info;
  int ret;

  /* We do not support sending signals to process groups */

  if (pid <= 0)
    {
      return -ENOSYS;
    }

  /* Make sure that the signal is valid */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Keep things stationary through the following */

  sched_lock();

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_USER;
  info.si_errno           = EINTR;
  info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Send the signal */

  ret = nxsig_dispatch(pid, &info);

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: kill
 *
 * Description:
 *   The kill() system call can be used to send any signal to any task
 *   group.
 *
 *   Limitation: Sending of signals to 'process groups' is not
 *   supported in NuttX
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX kill
 *     specification encodes process group information as zero and
 *     negative pid values.  Only positive, non-zero values of pid are
 *     supported by this implementation.
 *   signo - The signal number to send.  If signo is zero, no signal is
 *     sent, but all error checking is performed.
 *
 * Returned Value:
 *    This is a standard POSIX application interface.  On success (at least
 *    one signal was sent), zero (OK) is returned.  On any failure , -1
 *    (ERROR) is returned, and errno is set appropriately:
 *
 *      EINVAL An invalid signal was specified.
 *      EPERM  The process does not have permission to send the
 *             signal to any of the target processes.
 *      ESRCH  The pid or process group does not exist.
 *      ENOSYS Do not support sending signals to process groups.
 *
 ****************************************************************************/

int kill(pid_t pid, int signo)
{
  int ret;

  /* Let nxsig_kill() do all of the work */

  ret = nxsig_kill(pid, signo);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
