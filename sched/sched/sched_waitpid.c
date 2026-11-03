/****************************************************************************
 * sched/sched/sched_waitpid.c
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

#include <sys/wait.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef CONFIG_SCHED_WAITPID

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_waitpid
 *
 * Description:
 *   This functions will obtain status information pertaining to one
 *   of the caller's child processes. This function will suspend
 *   execution of the calling thread until status information for one of the
 *   terminated child processes of the calling process is available, or until
 *   delivery of a signal whose action is either to execute a signal-catching
 *   function or to terminate the process. If more than one thread is
 *   suspended in nxsched_waitpid() awaiting termination of the same process,
 *   exactly one thread will return the process status at the time of the
 *   target process termination. If status information is available prior to
 *   the call to nxsched_waitpid(), return will be immediate.
 *
 * Input Parameters:
 *   pid - The task ID of the thread to waid for
 *   stat_loc - The location to return the exit status
 *   options - Modifiable behavior, see sys/wait.h.
 *
 * Returned Value:
 *   If nxsched_waitpid() returns because the status of a child process is
 *   available, it will return a value equal to the process ID of the child
 *   process for which status is reported.
 *
 *   If nxsched_waitpid() returns due to the delivery of a signal to the
 *   calling process, -1 will be returned and errno set to EINTR.
 *
 *   If nxsched_waitpid() was invoked with WNOHANG set in options, it has
 *   at least one child process specified by pid for which status is not
 *   available, and status is not available for any process specified by
 *   pid, 0 is returned.
 *
 *   Otherwise, (pid_t)-1 will be returned, and errno set to indicate the
 *   error:
 *
 *   ECHILD - The process specified by pid does not exist or is not a child
 *            of the calling process, or the process group specified by pid
 *            does not exist does not have any member process that is a child
 *            of the calling process.
 *   EINTR - The function was interrupted by a signal. The value of the
 *           location pointed to by stat_loc is undefined.
 *   EINVAL - The options argument is not valid.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
/****************************************************************************
 *
 * If CONFIG_SCHED_HAVE_PARENT is defined, then waitpid() is waitid() with
 * a status in place of the siginfo: the pid argument gives the idtype and
 * the id, and WEXITED is always set.
 *
 ****************************************************************************/

pid_t nxsched_waitpid(pid_t pid, FAR int *stat_loc, int options)
{
  siginfo_t info;
  idtype_t idtype;
  pid_t id;
  int ret;

  if (pid == INVALID_PROCESS_ID)
    {
      idtype = P_ALL;
      id     = 0;
    }
  else if (pid > 0)
    {
      idtype = P_PID;
      id     = pid;
    }
  else
    {
      /* 0 is the process group of the caller */

      idtype = P_PGID;
      id     = -pid;
    }

  ret = nxsched_waitid(idtype, id, &info, options | WEXITED);
  if (ret < 0)
    {
      return ret;
    }

  /* WNOHANG and no status */

  if (info.si_pid == 0)
    {
      return 0;
    }

  if (stat_loc != NULL)
    {
      if (info.si_code == CLD_STOPPED)
        {
          *stat_loc = WAIT_STOPPED(info.si_status);
        }
      else if (info.si_code == CLD_CONTINUED)
        {
          *stat_loc = WAIT_CONTINUED;
        }
      else
        {
          *stat_loc = info.si_status << 8;
        }
    }

  return info.si_pid;
}
#else
pid_t nxsched_waitpid(pid_t pid, FAR int *stat_loc, int options)
{
  FAR struct tcb_s *ctcb;
  FAR struct task_group_s *group;
  bool mystat = false;
  irqstate_t flags;
  int ret = -ECHILD;

  /* NOTE: sched_lock() is not enough for SMP
   * because the child task is running on another CPU
   */

  flags = enter_critical_section();

  /* Get the TCB corresponding to this PID */

  ctcb = nxsched_get_tcb(pid);
  if (ctcb != NULL)
    {
      if ((group = ctcb->group) != NULL &&
          (ctcb->flags & TCB_FLAG_EXIT_PROCESSING) == 0)
        {
          /* Lock this group so that it cannot be
           * deleted until the wait completes
           */

          group_add_waiter(group);

          /* "If more than one thread is suspended in waitpid() awaiting
           * termination of the same process, exactly one thread will
           * return the process status at the time of the target process
           * termination."  Hmmm.. what do we return to the others?
           */

          if (group->tg_waitflags == 0)
            {
              /* Save the waitpid() options, setting the non-standard
               * WCLAIMED bit to assure that tg_waitflags is non-zero.
               */

              group->tg_waitflags = (uint8_t)options | WCLAIMED;

              /* Save the return status location (which may be NULL) */

              group->tg_statloc = stat_loc;

              /* We are the waipid() instance
               * that gets the return status
               */

              mystat = true;
            }

          /* Then wait for the task to exit */

          if ((options & WNOHANG) != 0)
            {
              /* Don't wait if status is not available */

              ret = nxsem_trywait(&group->tg_exitsem);
            }
          else
            {
              /* Wait if necessary for status to become available */

              ret = nxsem_wait(&group->tg_exitsem);
            }

          group_del_waiter(group);

          if (ret < 0)
            {
              /* Handle the awkward case of whether or not we
               * need to nullify the stat_loc value.
               */

              if (mystat)
                {
                  group->tg_statloc   = NULL;
                  group->tg_waitflags = 0;
                }

              if ((options & WNOHANG) != 0)
                {
                  pid = 0;
                }
              else
                {
                  pid = ret;
                }
            }

          /* On success, return the PID */

          ret = pid;
        }
    }

  leave_critical_section(flags);

  return ret;
}
#endif /* CONFIG_SCHED_HAVE_PARENT */

/****************************************************************************
 * Name: waitpid
 *
 * Description:
 *   The waitpid() functions will obtain status information pertaining to one
 *   of the caller's child processes. The waitpid() function will suspend
 *   execution of the calling thread until status information for one of the
 *   terminated child processes of the calling process is available, or until
 *   delivery of a signal whose action is either to execute a signal-catching
 *   function or to terminate the process. If more than one thread is
 *   suspended in waitpid() awaiting termination of the same process, exactly
 *   one thread will return the process status at the time of the target
 *   process termination. If status information is available prior to the
 *   call to waitpid(), return will be immediate.
 *
 *   The pid argument specifies a set of child processes for which status is
 *   requested. The waitpid() function will only return the status of a child
 *   process from this set:
 *
 *   - If pid is equal to (pid_t)-1, status is requested for any child
 *     process. In this respect, waitpid() is then equivalent to wait().
 *   - If pid is greater than 0, it specifies the process ID of a single
 *     child process for which status is requested.
 *   - If pid is 0, status is requested for any child process whose process
 *     group ID is equal to that of the calling process.
 *   - If pid is less than (pid_t)-1, status is requested for any child
 *     process whose process group ID is equal to the absolute value of pid.
 *
 *   The options argument is constructed from the bitwise-inclusive OR of
 *   zero or more of the following flags, defined in the <sys/wait.h> header:
 *
 *   WCONTINUED - The waitpid() function will report the status of any
 *     continued child process specified by pid whose status has not been
 *     reported since it continued from a job control stop.
 *   WNOHANG - The waitpid() function will not suspend execution of the
 *    calling thread if status is not immediately available for one of the
 *    child processes specified by pid.
 *   WUNTRACED - The status of any child processes specified by pid that are
 *    stopped, and whose status has not yet been reported since they stopped,
 *    will also be reported to the requesting process.
 *
 *   If the calling process has SA_NOCLDWAIT set or has SIGCHLD set to
 *   SIG_IGN, and the process has no unwaited-for children that were
 *   transformed into zombie processes, the calling thread will block until
 *   all of the children of the process containing the calling thread
 *   terminate, and waitpid() will fail and set errno to ECHILD.
 *
 *   If waitpid() returns because the status of a child process is available,
 *   these functions will return a value equal to the process ID of the child
 *   process. In this case, if the value of the argument stat_loc is not a
 *   null pointer, information will be stored in the location pointed to by
 *   stat_loc. The value stored at the location pointed to by stat_loc will
 *   be 0 if and only if the status returned is from a terminated child
 *   process that terminated by one of the following means:
 *
 *   1. The process returned 0 from main().
 *   2. The process called _exit() or exit() with a status argument of 0.
 *   3. The process was terminated because the last thread in the process
 *      terminated.
 *
 *   Regardless of its value, this information may be interpreted using the
 *   following macros, which are defined in <sys/wait.h> and evaluate to
 *   integral expressions; the stat_val argument is the integer value pointed
 *   to by stat_loc.
 *
 *   WIFEXITED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that terminated normally.
 *   WEXITSTATUS(stat_val) - If the value of WIFEXITED(stat_val) is non-zero,
 *     this macro evaluates to the low-order 8 bits of the status argument
 *     that the child process passed to _exit() or exit(), or the value the
 *     child process returned from main().
 *   WIFSIGNALED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that terminated due to the receipt of a
 *     signal that was not caught (see <signal.h>).
 *   WTERMSIG(stat_val)  - If the value of WIFSIGNALED(stat_val) is non-zero,
 *     this macro evaluates to the number of the signal that caused the
 *     termination of the child process.
 *   WIFSTOPPED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that is currently stopped.
 *   WSTOPSIG(stat_val) - If the value of WIFSTOPPED(stat_val) is non-zero,
 *     this macro evaluates to the number of the signal that caused the child
 *     process to stop.
 *   WIFCONTINUED(stat_val) - Evaluates to a non-zero value if status was
 *    returned for a child process that has continued from a job control
 *    stop.
 *
 * Input Parameters:
 *   pid - The task ID of the thread to waid for
 *   stat_loc - The location to return the exit status
 *   options - Modifiable behavior, see sys/wait.h.
 *
 * Returned Value:
 *   If waitpid() returns because the status of a child process is available,
 *   it will return a value equal to the process ID of the child process for
 *   which status is reported.
 *
 *   If waitpid() returns due to the delivery of a signal to the calling
 *   process, -1 will be returned and errno set to EINTR.
 *
 *   If waitpid() was invoked with WNOHANG set in options, it has at least
 *   one child process specified by pid for which status is not available,
 *   and status is not available for any process specified by pid, 0 is
 *   returned.
 *
 *   Otherwise, (pid_t)-1 will be returned, and errno set to indicate the
 *   error:
 *
 *   ECHILD - The process specified by pid does not exist or is not a child
 *            of the calling process, or the process group specified by pid
 *            does not exist does not have any member process that is a child
 *            of the calling process.
 *   EINTR - The function was interrupted by a signal. The value of the
 *           location pointed to by stat_loc is undefined.
 *   EINVAL - The options argument is not valid.
 *
 * Assumptions:
 *
 * Compatibility
 *   If there is no SIGCHLD signal supported (CONFIG_SCHED_HAVE_PARENT not
 *   defined), then waitpid() is still available, but does not obey the
 *   restriction that the pid be a child of the caller.
 *
 ****************************************************************************/

pid_t waitpid(pid_t pid, FAR int *stat_loc, int options)
{
  pid_t ret;

  /* waitpid() is a cancellation point */

  enter_cancellation_point();

  /* Let nxsched_waitpid() do the work. */

  ret = nxsched_waitpid(pid, stat_loc, options);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_SCHED_WAITPID */
