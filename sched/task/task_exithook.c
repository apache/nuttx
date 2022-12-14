/****************************************************************************
 * sched/task/task_exithook.c
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

#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mm/mm.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"
#include "task/task.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_exitstatus
 *
 * Description:
 *   Report exit status when main task of a task group exits
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static inline void nxtask_exitstatus(FAR struct task_group_s *group,
                                     int status)
{
  FAR struct child_status_s *child;

  /* Check if the parent task group has suppressed retention of
   * child exit status information.
   */

  if ((group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
    {
      /* No.. Find the exit status entry for this task in the parent TCB */

      child = group_find_child(group, gettid());
      if (child)
        {
          /* Save the exit status..  For the case of HAVE_GROUP_MEMBERS,
           * the child status will be as exited until the last member
           * of the task group exits.
           */

          child->ch_status = status;
        }
    }
}
#else

#  define nxtask_exitstatus(group,status)

#endif /* CONFIG_SCHED_CHILD_STATUS */

/****************************************************************************
 * Name: nxtask_groupexit
 *
 * Description:
 *   Mark that the final thread of a child task group as exited.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static inline void nxtask_groupexit(FAR struct task_group_s *group)
{
  FAR struct child_status_s *child;

  /* Check if the parent task group has suppressed retention of child exit
   * status information.
   */

  if ((group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
    {
      /* No.. Find the exit status entry for this task in the parent TCB */

      child = group_find_child(group, gettid());
      if (child)
        {
          /* Mark that all members of the child task group has exited */

          child->ch_flags |= CHILD_FLAG_EXITED;
        }
    }
}

#else

#  define nxtask_groupexit(group)

#endif /* CONFIG_SCHED_CHILD_STATUS */

/****************************************************************************
 * Name: nxtask_sigchild
 *
 * Description:
 *   Send the SIGCHLD signal to the parent thread
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
#ifdef HAVE_GROUP_MEMBERS
static inline void nxtask_sigchild(pid_t ppid, FAR struct tcb_s *ctcb,
                                   int status)
{
  FAR struct task_group_s *chgrp = ctcb->group;
  FAR struct task_group_s *pgrp;
  siginfo_t info;

  DEBUGASSERT(chgrp);

  /* Get the parent task group.  It is possible that all of the members of
   * the parent task group have exited.  This would not be an error.  In
   * this case, the child task group has been orphaned.
   */

  pgrp = group_findbypid(ppid);
  if (!pgrp)
    {
      /* Set the task group ID to an invalid group ID.  The dead parent
       * task group ID could get reused some time in the future.
       */

      chgrp->tg_ppid = INVALID_PROCESS_ID;
      return;
    }

  /* Save the exit status now if this is the main thread of the task group
   * that is exiting. Only the exiting main task of a task group carries
   * interpretable exit  Check if this is the main task that is exiting.
   */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((ctcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
    {
      nxtask_exitstatus(pgrp, status);
    }

  /* But only the final exiting thread in a task group, whatever it is,
   * should generate SIGCHLD.
   */

  if (chgrp->tg_nmembers == 1)
    {
      /* Mark that all of the threads in the task group have exited */

      nxtask_groupexit(pgrp);

      /* Create the siginfo structure.  We don't actually know the cause.
       * That is a bug. Let's just say that the child task just exited
       * for now.
       */

      info.si_signo           = SIGCHLD;
      info.si_code            = CLD_EXITED;
      info.si_errno           = OK;
      info.si_value.sival_ptr = NULL;
      info.si_pid             = chgrp->tg_pid;
      info.si_status          = status;

      /* Send the signal to one thread in the group */

      group_signal(pgrp, &info);
    }
}

#else /* HAVE_GROUP_MEMBERS */

static inline void nxtask_sigchild(FAR struct tcb_s *ptcb,
                                   FAR struct tcb_s *ctcb, int status)
{
  siginfo_t info;

  /* If task groups are not supported then we will report SIGCHLD when the
   * task exits.  Unfortunately, there could still be threads in the group
   * that are still running.
   */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((ctcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Save the exit status now of the main thread */

      nxtask_exitstatus(ptcb->group, status);

#else /* CONFIG_SCHED_CHILD_STATUS */
      /* Exit status is not retained.  Just decrement the number of
       * children from this parent.
       */

      DEBUGASSERT(ptcb->group != NULL && ptcb->group->tg_nchildren > 0);
      ptcb->group->tg_nchildren--;

#endif /* CONFIG_SCHED_CHILD_STATUS */

      /* Create the siginfo structure.  We don't actually know the cause.
       * That is a bug. Let's just say that the child task just exited
       * for now.
       */

      info.si_signo           = SIGCHLD;
      info.si_code            = CLD_EXITED;
      info.si_errno           = OK;
      info.si_value.sival_ptr = NULL;
      info.si_pid             = ctcb->group->tg_pid;
      info.si_status          = status;

      /* Send the signal.  We need to use this internal interface so that we
       * can provide the correct si_code value with the signal.
       */

      nxsig_tcbdispatch(ptcb, &info);
    }
}

#endif /* HAVE_GROUP_MEMBERS */
#else /* CONFIG_SCHED_HAVE_PARENT */

#  define nxtask_sigchild(x,ctcb,status)

#endif /* CONFIG_SCHED_HAVE_PARENT */

/****************************************************************************
 * Name: nxtask_signalparent
 *
 * Description:
 *   Send the SIGCHLD signal to the parent task group
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
static inline void nxtask_signalparent(FAR struct tcb_s *ctcb, int status)
{
#ifdef HAVE_GROUP_MEMBERS
  DEBUGASSERT(ctcb && ctcb->group);

  /* Keep things stationary throughout the following */

  sched_lock();

  /* Send SIGCHLD to all members of the parent's task group */

  nxtask_sigchild(ctcb->group->tg_ppid, ctcb, status);
  sched_unlock();
#else
  FAR struct tcb_s *ptcb;

  /* Keep things stationary throughout the following */

  sched_lock();

  /* Get the TCB of the receiving, parent task.  We do this early to
   * handle multiple calls to nxtask_signalparent.
   */

  ptcb = nxsched_get_tcb(ctcb->group->tg_ppid);
  if (ptcb == NULL)
    {
      /* The parent no longer exists... bail */

      sched_unlock();
      return;
    }

  /* Send SIGCHLD to all members of the parent's task group.  NOTE that the
   * SIGCHLD signal is only sent once either (1) if this is the final thread
   * of the task group that is exiting (HAVE_GROUP_MEMBERS) or (2) if the
   * main thread of the group is exiting (!HAVE_GROUP_MEMBERS).
   */

  nxtask_sigchild(ptcb, ctcb, status);
  sched_unlock();
#endif
}
#else
#  define nxtask_signalparent(ctcb,status)
#endif

/****************************************************************************
 * Name: nxtask_exitwakeup
 *
 * Description:
 *   Wakeup any tasks waiting for this task to exit
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
static inline void nxtask_exitwakeup(FAR struct tcb_s *tcb, int status)
{
  FAR struct task_group_s *group = tcb->group;

  /* Have we already left the group? */

  if (group)
    {
      /* Only tasks (and kernel threads) return valid status.  Record the
       * exit status when the task exists.  The group, however, may still
       * be executing.
       */

#ifndef CONFIG_DISABLE_PTHREAD
      if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
        {
          /* Report the exit status.  We do not nullify tg_statloc here
           * because we want to prevent other tasks from registering for
           * the return status.  There is only one task per task group,
           * there for, this logic should execute exactly once in the
           * lifetime of the task group.
           *
           * "If more than one thread is suspended in waitpid() awaiting
           *  termination of the same process, exactly one thread will
           *  return the process status at the time of the target process
           *  termination."
           *
           *  Hmmm.. what do we return to the others?
           */

          if (group->tg_statloc != NULL)
            {
              *group->tg_statloc = status << 8;
            }
        }

      /* Is this the last thread in the group? */

      if (group->tg_nmembers == 1)
        {
          /* Yes.. Wakeup any tasks waiting for this task to exit */

          group->tg_statloc   = NULL;
          group->tg_waitflags = 0;

          while (group->tg_exitsem.semcount < 0)
            {
              /* Wake up the thread */

              nxsem_post(&group->tg_exitsem);
            }
        }
    }
}
#else
#  define nxtask_exitwakeup(tcb, status)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_exithook
 *
 * Description:
 *   This function implements some of the internal logic of exit() and
 *   task_delete().  This function performs some clean-up and other actions
 *   required when a task exits:
 *
 *   - All open streams are flushed and closed.
 *   - All functions registered with atexit() and on_exit() are called, in
 *     the reverse order of their registration.
 *
 *   When called from exit(), the tcb still resides at the head of the ready-
 *   to-run list.  The following logic is safe because we will not be
 *   returning from the exit() call.
 *
 *   When called from nxtask_terminate() we are operating on a different
 *   thread; on the thread that called task_delete().  In this case,
 *   task_delete will have already removed the tcb from the ready-to-run
 *   list to prevent any further action on this task.
 *
 ****************************************************************************/

void nxtask_exithook(FAR struct tcb_s *tcb, int status)
{
  /* Under certain conditions, nxtask_exithook() can be called multiple
   * times.  A bit in the TCB was set the first time this function was
   * called.  If that bit is set, then just exit doing nothing more..
   */

  if ((tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      return;
    }

#ifdef CONFIG_CANCELLATION_POINTS
  /* Mark the task as non-cancelable to avoid additional calls to exit()
   * due to any cancellation point logic that might get kicked off by
   * actions taken during exit processing.
   */

  tcb->flags  |= TCB_FLAG_NONCANCELABLE;
  tcb->flags  &= ~TCB_FLAG_CANCEL_PENDING;
  tcb->cpcount = 0;
#endif

  /* If the task was terminated by another task, it may be in an unknown
   * state.  Make some feeble effort to recover the state.
   */

  nxtask_recover(tcb);

  /* NOTE: signal handling needs to be done in a criticl section */

#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#endif

  /* Send the SIGCHLD signal to the parent task group */

  nxtask_signalparent(tcb, status);

  /* Wakeup any tasks waiting for this task to exit */

  nxtask_exitwakeup(tcb, status);

  /* Leave the task group.  Perhaps discarding any un-reaped child
   * status (no zombies here!)
   */

  group_leave(tcb);

  /* Deallocate anything left in the TCB's queues */

  nxsig_cleanup(tcb); /* Deallocate Signal lists */

#ifdef CONFIG_SCHED_DUMP_LEAK
  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      kmm_memdump(tcb->pid);
    }
  else
    {
      umm_memdump(tcb->pid);
    }
#endif

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif

  /* This function can be re-entered in certain cases.  Set a flag
   * bit in the TCB to not that we have already completed this exit
   * processing.
   */

  tcb->flags |= TCB_FLAG_EXIT_PROCESSING;
}
