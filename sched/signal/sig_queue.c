/****************************************************************************
 * sched/signal/sig_queue.c
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

int nxsig_queue(int pid, int signo, union sigval value)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct tcb_s *rtcb = this_task();
#endif
  siginfo_t info;
  int ret;

  sinfo("pid=0x%08x signo=%d value=%d\n", pid, signo, value.sival_int);

  /* Sanity checks */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_QUEUE;
  info.si_errno           = OK;
  info.si_value           = value;
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
 *    any failure, -1 (ERROR) is returned and errno variable is set
 *    appropriately:
 *
 *    EGAIN  - The limit of signals which may be queued has been reached.
 *    EINVAL - sig was invalid.
 *    EPERM  - The  process  does  not  have  permission to send the
 *             signal to the receiving process.
 *    ESRCH  - No process has a PID matching pid.
 *
 ****************************************************************************/

int sigqueue(int pid, int signo, union sigval value)
{
  int ret;

  /* Let nxsig_queue() do all of the real work */

  ret = nxsig_queue(pid, signo, value);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
