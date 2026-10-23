/****************************************************************************
 * sched/signal/sig_kill.c
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

#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <nuttx/debug.h>

#include <nuttx/signal.h>

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
 *   A process group is one process in NuttX: a process group ID is the
 *   ID of its only process.  A signal to all processes (pid -1) is not
 *   supported.
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX nxsig_kill
 *     specification encodes process group information as zero and
 *     negative pid values: 0 is the caller and -pid is the process pid.
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
 *    ENOSYS pid is -1: a signal to all processes is not supported.
 *
 ****************************************************************************/

int nxsig_kill(pid_t pid, int signo)
{
#if !defined(CONFIG_DISABLE_ALL_SIGNALS) && defined(CONFIG_SCHED_HAVE_PARENT)
  FAR struct tcb_s *rtcb = this_task();
#endif
#ifndef CONFIG_DISABLE_ALL_SIGNALS
  siginfo_t info;
#endif

  /* NuttX has no process group with more than one process: the process
   * group of a process is the process itself (getpgid(), setpgid() and
   * killpg() say so).  So pid 0 (the process group of the caller) is the
   * caller, and -pid (the process group pid) is the process pid.  All the
   * processes of the system (-1) are not supported.
   */

  if (pid == 0)
    {
      pid = nxsched_getpid();
    }
  else if (pid == -1)
    {
      return -ENOSYS;
    }
  else if (pid < 0 && pid != INT_MIN)
    {
      /* The process pid.  INT_MIN stays as it is (-INT_MIN does not
       * fit): no process has it, so the signal fails with ESRCH.
       */

      pid = -pid;
    }

  if (signo == 0)
    {
      return (nxsched_get_tcb(pid) != NULL) ? 0 : -ESRCH;
    }

#ifdef CONFIG_DISABLE_ALL_SIGNALS
  return -ENOSYS;
#else
  /* Make sure that the signal is valid */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_USER;
  info.si_errno           = EINTR;
  info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_uid             = getuid();
  info.si_status          = OK;
#endif

  /* Send the signal */

  return nxsig_dispatch(pid, &info, false);
#endif
}

/****************************************************************************
 * Name: kill
 *
 * Description:
 *   The kill() system call can be used to send any signal to any task
 *   group.
 *
 *   A process group is one process in NuttX: a process group ID is the
 *   ID of its only process.  A signal to all processes (pid -1) is not
 *   supported.
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX kill
 *     specification encodes process group information as zero and
 *     negative pid values: 0 is the caller and -pid is the process pid.
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
 *      ENOSYS pid is -1: a signal to all processes is not supported.
 *
 ****************************************************************************/

int kill(pid_t pid, int signo)
{
  int ret;

  if (signo != 0 && !nxsched_may_control(pid))
    {
      set_errno(EPERM);
      return ERROR;
    }

  /* Let nxsig_kill() do all of the work */

  ret = nxsig_kill(pid, signo);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
