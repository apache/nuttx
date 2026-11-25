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

#include <string.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_SCHED_PROCESS_GROUPS
struct nxsig_pgrp_s
{
  pid_t pgid;   /* The process group ID */
  int   signo;  /* The signal number, or 0 */
  bool  check;  /* Signal only the processes that the caller may control */
  int   ret;    /* OK if one process got the signal, else the last error */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_PROCESS_GROUPS
/****************************************************************************
 * Name: nxsig_kill_member
 *
 * Description:
 *   The nxsched_foreach() function of nxsig_kill_pgrp().
 *
 ****************************************************************************/

static void nxsig_kill_member(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct nxsig_pgrp_s *pgrp = arg;
  int ret;

  if (!group_in_pgrp(tcb, pgrp->pgid))
    {
      return;
    }

  if (pgrp->check && !nxsched_may_control(tcb->pid))
    {
      ret = -EPERM;
    }
  else
    {
      ret = nxsig_kill(tcb->pid, pgrp->signo);
    }

  if (pgrp->ret != OK)
    {
      pgrp->ret = ret;
    }
}

/****************************************************************************
 * Name: nxsig_kill_pgrp
 *
 * Description:
 *   Send a signal to each process of a process group.  pid is the pid of
 *   kill(): 0 is the process group of the caller, -pid the process group
 *   pid.  If check is true, only the processes that the caller may control
 *   get the signal.
 *
 ****************************************************************************/

static int nxsig_kill_pgrp(pid_t pid, int signo, bool check)
{
  struct nxsig_pgrp_s pgrp;

  if (signo != 0 && !GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

  pgrp.pgid  = pid == 0 ? this_task()->group->tg_pgid : -pid;
  pgrp.signo = signo;
  pgrp.check = check;
  pgrp.ret   = -ESRCH;
  nxsched_foreach(nxsig_kill_member, &pgrp);
  return pgrp.ret;
}
#endif

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
 *   With CONFIG_SCHED_PROCESS_GROUPS, each process of the process group
 *   gets the signal.  Without it, a process group is one process: a
 *   process group ID is the ID of its only process.  A signal to all
 *   processes (pid -1) is not supported.
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX nxsig_kill
 *     specification encodes process group information as zero and
 *     negative pid values: 0 is the process group of the caller and -pid
 *     is the process group pid.
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
  int ret = -EINVAL;
#endif

#ifdef CONFIG_SCHED_PROCESS_GROUPS
  /* pid 0 is the process group of the caller, -pid the process group pid.
   * INT_MIN is no process group (-INT_MIN does not fit).
   */

  if (pid == 0 || (pid < -1 && pid != INT_MIN))
    {
      return nxsig_kill_pgrp(pid, signo, false);
    }
#endif

  /* Without process groups, a process group is one process: the process
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

  if (GOOD_SIGNO(signo))
    {
      /* Zero the whole siginfo first: sigaction-queued signals carry a
       * si_user field that only the sigaction queue path assigns, so a
       * stack-garbage si_user here would be memcpy'd into the pending
       * entry and handed back to a sigtimedwait() consumer.
       */

      memset(&info, 0, sizeof(siginfo_t));
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

      ret = nxsig_dispatch(pid, &info, false, NULL, false);
    }

  return ret;
#endif
}

/****************************************************************************
 * Name: nxsig_kill_foreground
 *
 * Description:
 *   Send a signal of a terminal to its foreground process group pgrp.  If
 *   no process is in the process group pgrp, the process pgrp gets it:
 *   the TIOCSCTTY ioctl of NSH saves the ID of a process.
 *
 * Returned Value:
 *   As nxsig_kill().
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_PROCESS_GROUPS
int nxsig_kill_foreground(pid_t pgrp, int signo)
{
  int ret;

  if (pgrp > 0)
    {
      ret = nxsig_kill_pgrp(-pgrp, signo, false);
      if (ret != -ESRCH)
        {
          return ret;
        }
    }

  return nxsig_kill(pgrp, signo);
}
#endif

/****************************************************************************
 * Name: kill
 *
 * Description:
 *   The kill() system call can be used to send any signal to any task
 *   group.
 *
 *   With CONFIG_SCHED_PROCESS_GROUPS, each process of the process group
 *   gets the signal.  Without it, a process group is one process: a
 *   process group ID is the ID of its only process.  A signal to all
 *   processes (pid -1) is not supported.
 *
 * Input Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX kill
 *     specification encodes process group information as zero and
 *     negative pid values: 0 is the process group of the caller and -pid
 *     is the process group pid.
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

#ifdef CONFIG_SCHED_PROCESS_GROUPS
  /* A process group: nxsig_kill_pgrp() checks each process */

  if (pid == 0 || (pid < -1 && pid != INT_MIN))
    {
      ret = nxsig_kill_pgrp(pid, signo, signo != 0);
      if (ret < 0)
        {
          set_errno(-ret);
          ret = ERROR;
        }

      return ret;
    }
#endif

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
