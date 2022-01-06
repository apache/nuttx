/****************************************************************************
 * sched/sched/sched_waitid.c
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

#include "sched/sched.h"
#include "group/group.h"

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exited_child
 *
 * Description:
 *   Handle the case where a child exited properlay was we (apparently) lost
 *   the detch of child signal.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static void exited_child(FAR struct tcb_s *rtcb,
                         FAR struct child_status_s *child,
                         FAR siginfo_t *info)
{
  /* The child has exited. Return the saved exit status (and some fudged
   * information).
   */

  if (info)
    {
      info->si_signo           = SIGCHLD;
      info->si_code            = CLD_EXITED;
      info->si_errno           = OK;
      info->si_value.sival_ptr = NULL;
      info->si_pid             = child->ch_pid;
      info->si_status          = child->ch_status;
    }

  /* Discard the child entry */

  group_remove_child(rtcb->group, child->ch_pid);
  group_free_child(child);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_waitid
 ****************************************************************************/

int nx_waitid(int idtype, id_t id, FAR siginfo_t *info, int options)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *ctcb;
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
  bool retains;
#endif
  sigset_t set;
  int ret = OK;

  /* MISSING LOGIC:   If WNOHANG is provided in the options, then this
   * function should returned immediately.  However, there is no mechanism
   * available now know if the thread has child:  The children remember
   * their parents (if CONFIG_SCHED_HAVE_PARENT) but the parents do not
   * remember their children.
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Only ID types P_PID and P_ALL are supported */

  if (idtype != P_PID && idtype != P_ALL)
    {
      return -ENOSYS;
    }

  /* None of the options are supported except for WEXITED (which must be
   * provided.  Currently SIGCHLD always reports CLD_EXITED so we cannot
   * distinguish any other events.
   */

  if ((options & WEXITED) == 0)
    {
      return -ENOSYS;
    }

  if ((options & ~(WEXITED | WNOHANG)) != 0)
    {
      return -ENOSYS;
    }
#endif

  /* Create a signal set that contains only SIGCHLD */

  sigemptyset(&set);
  nxsig_addset(&set, SIGCHLD);

  /* NOTE: sched_lock() is not enough for SMP
   * because the child task is running on another CPU
   */

#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#else
  /* Disable pre-emption so that nothing changes while the loop executes */

  sched_lock();
#endif

  /* Verify that this task actually has children and that the requested
   * TCB is actually a child of this task.
   */

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Does this task retain child status? */

  retains = ((rtcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0);

  if (rtcb->group->tg_children == NULL && retains)
    {
      /* There are no children */

      ret = -ECHILD;
      goto errout;
    }
  else if (idtype == P_PID)
    {
      /* Get the TCB corresponding to this PID and make sure that the
       * thread it is our child.
       */

      ctcb = nxsched_get_tcb((pid_t)id);
      if (ctcb && ctcb->group)
        {
          /* Make sure that the thread it is our child. */

          if (ctcb->group->tg_ppid != rtcb->pid)
            {
              ret = -ECHILD;
              goto errout;
            }
        }

      /* Does this task retain child status? */

      if (retains)
        {
          /* Check if this specific pid has allocated child status? */

          if (group_find_child(rtcb->group, (pid_t)id) == NULL)
            {
              /* This specific pid is not a child */

              ret = -ECHILD;
              goto errout;
            }
        }
    }
#else
  /* Child status is not retained. */

  if (rtcb->group->tg_nchildren == 0)
    {
      /* There are no children */

      ret = -ECHILD;
      goto errout;
    }
  else if (idtype == P_PID)
    {
      /* Get the TCB corresponding to this PID and make sure that the
       * thread is our child.
       */

      ctcb = nxsched_get_tcb((pid_t)id);

      if (!ctcb || !ctcb->group || ctcb->group->tg_ppid != rtcb->pid)
        {
          ret = -ECHILD;
          goto errout;
        }
    }
#endif

  /* Loop until the child that we are waiting for dies */

  for (; ; )
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      DEBUGASSERT(!retains || rtcb->group->tg_children);
      if (idtype == P_ALL)
        {
          /* We are waiting for any child to exit */

          if (retains && (child = group_exit_child(rtcb->group)) != NULL)
            {
              /* A child has exited.  Apparently we missed the signal.
               * Return the exit status and break out of the loop.
               */

              exited_child(rtcb, child, info);
              break;
            }
        }

      /* We are waiting for a specific PID.  Does this task retain child
       * status?
       */

      else if (retains)
        {
          /* Yes ... Get the current status of the child task. */

          child = group_find_child(rtcb->group, (pid_t)id);
          DEBUGASSERT(child);

          /* Did the child exit? */

          if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
            {
              /* The child has exited. Return the exit status and break out
               * of the loop.
               */

              exited_child(rtcb, child, info);
              break;
            }
        }
      else
        {
          /* We can use nxsig_kill() with signal number 0 to determine if
           * that task is still alive.
           */

          ret = nxsig_kill((pid_t)id, 0);
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
#else
      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      if (rtcb->group->tg_nchildren == 0 ||
          (idtype == P_PID && (ret = nxsig_kill((pid_t)id, 0)) < 0))
        {
          /* We know that the child task was running okay we started,
           * so we must have lost the signal.  What can we do?
           * Let's return ECHILD.. that is at least informative.
           */

          ret = -ECHILD;
          goto errout;
        }
#endif

      if ((options & WNOHANG) != 0)
        {
          /* SUSv4 says:
           *
           * "If waitid() returns because WNOHANG was specified and status
           * is not available for any process specified by idtype and id,
           * then the si_signo and si_pid members of the structure pointed
           * to by infop shall be set to zero and the values of other
           * members of the structure are unspecified."
           */

          info->si_signo = 0;
          info->si_pid = 0;
          break;
        }

      /* Wait for any death-of-child signal */

      ret = nxsig_waitinfo(&set, info);
      if (ret < 0)
        {
          goto errout;
        }

      /* Make there this was SIGCHLD */

      if (info->si_signo == SIGCHLD)
        {
          /* Yes.. Are we waiting for the death of a specific child? */

          if (idtype == P_PID)
            {
              /* Was this the death of the thread we were waiting for? */

              if (info->si_pid == (pid_t)id)
                {
                  /* Yes... return success */

#ifdef CONFIG_SCHED_CHILD_STATUS
                  if (retains)
                    {
                      child = group_find_child(rtcb->group, info->si_pid);
                      DEBUGASSERT(child);

                      if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
                        {
                          exited_child(rtcb, child, NULL);
                        }
                    }
#endif

                  break;
                }
            }

          /* Are we waiting for any child to change state? */

          else if (idtype == P_ALL)
            {
              /* Return success */

#ifdef CONFIG_SCHED_CHILD_STATUS
                  if (retains)
                    {
                      child = group_find_child(rtcb->group, info->si_pid);

                      if (child &&
                          (child->ch_flags & CHILD_FLAG_EXITED) != 0)
                        {
                          exited_child(rtcb, child, NULL);
                        }
                    }
#endif

              break;
            }

          /* Other ID types are not supported */

          else /* if (idtype == P_PGID) */
            {
              ret = -ENOSYS;
              goto errout;
            }
        }
    }

errout:

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#else
  sched_unlock();
#endif

  return ret;
}

/****************************************************************************
 * Name: waitid
 *
 * Description:
 *   The waitid() function suspends the calling thread until one child of
 *   the process containing the calling thread changes state. It records the
 *   current state of a child in the structure pointed to by 'info'. If a
 *   child process changed state prior to the call to waitid(), waitid()
 *   returns immediately. If more than one thread is suspended in wait() or
 *   waitpid() waiting termination of the same process, exactly one thread
 *   will return the process status at the time of the target process
 *   termination
 *
 *   The idtype and id arguments are used to specify which children waitid()
 *   will wait for.
 *
 *     If idtype is P_PID, waitid() will wait for the child with a process
 *     ID equal to (pid_t)id.
 *
 *     If idtype is P_PGID, waitid() will wait for any child with a process
 *     group ID equal to (pid_t)id.
 *
 *     If idtype is P_ALL, waitid() will wait for any children and id is
 *     ignored.
 *
 *   The options argument is used to specify which state changes waitid()
 *   will will wait for. It is formed by OR-ing together one or more of the
 *   following flags:
 *
 *     WEXITED - Wait for processes that have exited.
 *     WSTOPPED - Status will be returned for any child that has stopped
 *       upon receipt of a signal.
 *     WCONTINUED - Status will be returned for any child that was stopped
 *       and has been continued.
 *     WNOHANG - Return immediately if there are no children to wait for.
 *     WNOWAIT - Keep the process whose status is returned in 'info' in a
 *       waitable state. This will not affect the state of the process; the
 *       process may be waited for again after this call completes.
 *
 *   The 'info' argument must point to a siginfo_t structure. If waitid()
 *   returns because a child process was found that satisfied the conditions
 *   indicated by the arguments idtype and options, then the structure
 *   pointed to by 'info' will be filled in by the system with the status of
 *   the process. The si_signo member will always be equal to SIGCHLD.
 *
 * Input Parameters:
 *   See description.
 *
 * Returned Value:
 *   If waitid() returns due to the change of state of one of its children,
 *   0 is returned. Otherwise, -1 is returned and errno is set to indicate
 *   the error.
 *
 *   The waitid() function will fail if:
 *
 *     ECHILD - The calling process has no existing unwaited-for child
 *       processes.
 *     EINTR - The waitid() function was interrupted by a signal.
 *     EINVAL - An invalid value was specified for options, or idtype and id
 *       specify an invalid set of processes.
 *
 ****************************************************************************/

int waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options)
{
  int ret;

  /* waitid() is a cancellation point */

  enter_cancellation_point();

  ret = nx_waitid(idtype, id, info, options);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

#endif /* CONFIG_SCHED_WAITPID && CONFIG_SCHED_HAVE_PARENT */
