/****************************************************************************
 * sched/sched/sched_waitpid.c
 *
 *   Copyright (C) 2011-2013, 2015, 2017-2019 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:make
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

#include <sys/wait.h>
#include <signal.h>
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
 * Name: nx_waitpid
 ****************************************************************************/

#ifndef CONFIG_SCHED_HAVE_PARENT
pid_t nx_waitpid(pid_t pid, int *stat_loc, int options)
{
  FAR struct tcb_s *ctcb;
  FAR struct task_group_s *group;
  bool mystat = false;
  int ret;

  DEBUGASSERT(stat_loc);

  /* Disable pre-emption so that nothing changes in the following tests */

  sched_lock();

  /* Get the TCB corresponding to this PID */

  ctcb = nxsched_get_tcb(pid);
  if (ctcb == NULL)
    {
      ret = -ECHILD;
      goto errout;
    }

  /* Then the task group corresponding to this PID */

  group = ctcb->group;
  DEBUGASSERT(group);

  /* Lock this group so that it cannot be deleted until the wait completes */

  group_add_waiter(group);

  /* "If more than one thread is suspended in waitpid() awaiting termination
   * of the same process, exactly one thread will return the process status
   * at the time of the target process termination."  Hmmm.. what do we
   * return to the others?
   */

  if (group->tg_waitflags == 0)
    {
      /* Save the waitpid() options, setting the non-standard WCLAIMED bit to
       * assure that tg_waitflags is non-zero.
       */

      group->tg_waitflags = (uint8_t)options | WCLAIMED;

      /* Save the return status location (which may be NULL) */

      group->tg_statloc = stat_loc;

      /* We are the waipid() instance that gets the return status */

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
          goto errout;
        }
    }

  /* On success, return the PID */

  sched_unlock();
  return pid;

errout:
  sched_unlock();
  return ret;
}

/****************************************************************************
 *
 * If CONFIG_SCHED_HAVE_PARENT is defined, then waitpid will use the SIGHCLD
 * signal.  It can also handle the pid == (pid_t)-1 argument.  This is
 * slightly more spec-compliant.
 *
 * But then I have to be concerned about the fact that NuttX does not queue
 * signals.  This means that a flurry of signals can cause signals to be
 * lost (or to have the data in the struct siginfo to be overwritten by
 * the next signal).
 *
 ****************************************************************************/

#else
pid_t nx_waitpid(pid_t pid, int *stat_loc, int options)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *ctcb;
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child = NULL;
  bool retains;
#endif
  FAR struct siginfo info;
  sigset_t set;
  int ret;

  DEBUGASSERT(stat_loc);

  /* Create a signal set that contains only SIGCHLD */

  sigemptyset(&set);
  nxsig_addset(&set, SIGCHLD);

  /* Disable pre-emption so that nothing changes while the loop executes */

  sched_lock();

  /* Verify that this task actually has children and that the requested PID
   * is actually a child of this task.
   */

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Does this task retain child status? */

  retains = ((rtcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0);

  if (rtcb->group->tg_children == NULL && retains)
    {
      ret = -ECHILD;
      goto errout;
    }
  else if (pid != (pid_t)-1)
    {
      /* Get the TCB corresponding to this PID.  NOTE: If the child has
       * already exited, then the PID will not map to a valid TCB.
       */

      ctcb = nxsched_get_tcb(pid);
      if (ctcb != NULL)
        {
          /* Make sure that the thread it is our child. */

#ifdef HAVE_GROUP_MEMBERS
          if (ctcb->group->tg_pgrpid != rtcb->group->tg_grpid)
#else
          if (ctcb->group->tg_ppid != rtcb->pid)
#endif
            {
              ret = -ECHILD;
              goto errout;
            }
        }

      /* The child task is ours or it is no longer active.  Does the parent
       * task retain child status?
       */

      if (retains)
        {
          /* Yes.. Check if this specific pid has allocated child status? */

          if (group_find_child(rtcb->group, pid) == NULL)
            {
              ret = -ECHILD;
              goto errout;
            }
        }
    }

#else /* CONFIG_SCHED_CHILD_STATUS */

  if (rtcb->group->tg_nchildren == 0)
    {
      /* There are no children */

      ret = -ECHILD;
      goto errout;
    }
  else if (pid != (pid_t)-1)
    {
      /* Get the TCB corresponding to this PID and make sure that the
       * thread it is our child.
       */

      ctcb = nxsched_get_tcb(pid);

#ifdef HAVE_GROUP_MEMBERS
      if (ctcb == NULL || ctcb->group->tg_pgrpid != rtcb->group->tg_grpid)
#else
      if (ctcb == NULL || ctcb->group->tg_ppid != rtcb->pid)
#endif
        {
          ret = -ECHILD;
          goto errout;
        }
    }

#endif /* CONFIG_SCHED_CHILD_STATUS */

  /* Loop until the child that we are waiting for dies */

  for (; ; )
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      if (pid == (pid_t)-1)
        {
          /* We are waiting for any child, check if there are still
           * children.
           */

          DEBUGASSERT(!retains || rtcb->group->tg_children);
          if (retains && (child = group_exit_child(rtcb->group)) != NULL)
            {
              /* A child has exited.  Apparently we missed the signal.
               * Return the saved exit status.
               */

              /* The child has exited. Return the saved exit status */

              *stat_loc = child->ch_status << 8;

              /* Discard the child entry and break out of the loop */

              group_remove_child(rtcb->group, child->ch_pid);
              group_free_child(child);
              break;
            }
        }

      /* We are waiting for a specific PID. Does this task retain child
       * status?
       */

      else if (retains)
        {
          /* Get the current status of the child task. */

          child = group_find_child(rtcb->group, pid);
          DEBUGASSERT(child);

          /* Did the child exit? */

          if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
            {
              /* The child has exited. Return the saved exit status */

              *stat_loc = child->ch_status << 8;

              /* Discard the child entry and break out of the loop */

              group_remove_child(rtcb->group, pid);
              group_free_child(child);
              break;
            }
        }
      else
        {
          /* We can use nxsig_kill() with signal number 0 to determine if
           * that task is still alive.
           */

          ret = nxsig_kill(pid, 0);
          if (ret < 0)
            {
              /* It is no longer running.  We know that the child task
               * was running okay when we started, so we must have lost
               * the signal.  In this case, we know that the task exit'ed,
               * but we do not know its exit status.  It would be better
               * to reported ECHILD than bogus status.
               */

              ret = -ECHILD;
              goto errout;
            }
        }

#else  /* CONFIG_SCHED_CHILD_STATUS */

      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      if (rtcb->group->tg_nchildren == 0 ||
          (pid != (pid_t)-1 && nxsig_kill(pid, 0) < 0))
        {
          /* We know that the child task was running okay when we started,
           * so we must have lost the signal.  What can we do?
           * Let's return ECHILD.. that is at least informative.
           */

          ret = -ECHILD;
          goto errout;
        }

#endif /* CONFIG_SCHED_CHILD_STATUS */

      if ((options & WNOHANG) != 0)
        {
          pid = 0;
          break;
        }

      /* Wait for any death-of-child signal */

      ret = nxsig_waitinfo(&set, &info);
      if (ret < 0)
        {
          goto errout;
        }

      /* Was this the death of the thread we were waiting for? In the of
       * pid == (pid_t)-1, we are waiting for any child thread.
       */

      if (info.si_signo == SIGCHLD &&
         (pid == (pid_t)-1 || info.si_pid == pid))
        {
          /* Yes... return the status and PID (in the event it was -1) */

          *stat_loc = info.si_status << 8;
          pid = info.si_pid;

#ifdef CONFIG_SCHED_CHILD_STATUS
          if (retains)
            {
              /* Recover the exiting child */

              child = group_exit_child(rtcb->group);
              DEBUGASSERT(child != NULL);

              /* Discard the child entry, if we have one */

              if (child != NULL)
                {
                  group_remove_child(rtcb->group, child->ch_pid);
                  group_free_child(child);
                }
            }
#endif /* CONFIG_SCHED_CHILD_STATUS */

          break;
        }
    }

  sched_unlock();
  return pid;

errout:
  sched_unlock();
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
 *   options - ignored
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

pid_t waitpid(pid_t pid, int *stat_loc, int options)
{
  pid_t ret;

  /* waitpid() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_waitpid() do the work. */

  ret = nx_waitpid(pid, stat_loc, options);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_SCHED_WAITPID */
