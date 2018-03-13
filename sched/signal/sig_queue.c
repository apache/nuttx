/****************************************************************************
 * sched/signal/sig_queue.c
 *
 *   Copyright (C) 2007-2009, 2013, 2017 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <signal.h>
#include <debug.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_queue
 *
 * Description:
 *   This function sends the signal specified by signo with the signal
 *   parameter value to the process specified by pid.
 *
 *   If the receiving process has the signal blocked via the sigprocmask,
 *   the signal will pend until it is unmasked. Only one pending signal (per
 *   signo) is retained.  This is consistent with POSIX which states, "If
 *   a subsequent occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is delivered more than
 *   once.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sigqueue() except that it does not modify the errno value.
 *
 * Input Parameters:
 *   pid - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
int nxsig_queue (int pid, int signo, union sigval value)
#else
int nxsig_queue(int pid, int signo, void *sival_ptr)
#endif
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
#endif
  siginfo_t info;
  int ret;

#ifdef CONFIG_CAN_PASS_STRUCTS
  sinfo("pid=0x%08x signo=%d value=%d\n", pid, signo, value.sival_int);
#else
  sinfo("pid=0x%08x signo=%d value=%p\n", pid, signo, sival_ptr);
#endif

  /* Sanity checks */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_QUEUE;
  info.si_errno           = OK;
#ifdef CONFIG_CAN_PASS_STRUCTS
  info.si_value           = value;
#else
  info.si_value.sival_ptr = sival_ptr;
#endif
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Send the signal */

  sched_lock();
  ret = nxsig_dispatch(pid, &info);
  sched_unlock();

  return ret;
}

/****************************************************************************
 * Name: sigqueue
 *
 * Description:
 *   This function sends the signal specified by signo with the signal
 *   parameter value to the process specified by pid.
 *
 *   If the receiving process has the signal blocked via the sigprocmask,
 *   the signal will pend until it is unmasked. Only one pending signal (per
 *   signo) is retained.  This is consistent with POSIX which states, "If
 *   a subsequent occurrence of a pending signal is generated, it is
 *   implementation defined as to whether the signal is delivered more than
 *   once."
 *
 * Input Parameters:
 *   pid - Process ID of task to receive signal
 *   signo - Signal number
 *   value - Value to pass to task with signal
 *
 * Returned Value:
 *    On  success (at least one signal was sent), zero (OK) is returned.  On
 *    any failure, -1 (ERROR) is returned and errno varaible is set
 *    appropriately:
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
int sigqueue (int pid, int signo, union sigval value)
#else
int sigqueue(int pid, int signo, void *sival_ptr)
#endif
{
  int ret;

  /* Let nxsig_queue() do all of the real work */

#ifdef CONFIG_CAN_PASS_STRUCTS
  ret = nxsig_queue(pid, signo, value);
#else
  ret = nxsig_queue(pid, signo, sival_ptr);
#endif
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
