/****************************************************************************
 * sched/task/task_exithook.c
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

#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <nuttx/debug.h>
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
                                     FAR struct tcb_s *ctcb, int status)
{
  FAR struct child_status_s *child;

  /* Check if the parent task group has suppressed retention of
   * child exit status information.
   */

  if ((group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
    {
      /* No.. Find the exit status entry for this task in the parent TCB */

      child = group_find_child(group, ctcb->pid);
      if (child)
        {
          /* Save the exit status..  For the case of HAVE_GROUP_MEMBERS,
           * the child status will be as exited until the last member
           * of the task group exits.
           */

          child->ch_status = status;
#ifdef CONFIG_SCHED_USER_IDENTITY
          child->ch_uid    = ctcb->group->tg_uid;
#endif
        }
    }

  group->tg_exitcode = status;
}
#else

#  define nxtask_exitstatus(group,ctcb,status) (group)->tg_exitcode = (status);

#endif /* CONFIG_SCHED_CHILD_STATUS */

/****************************************************************************
 * Name: nxtask_groupexit
 *
 * Description:
 *   Mark that the final thread of a child task group as exited.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static inline void nxtask_groupexit(FAR struct task_group_s *group,
                                    pid_t pid)
{
  FAR struct child_status_s *child;

  /* Check if the parent task group has suppressed retention of child exit
   * status information.
   */

  if ((group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
    {
      /* No.. Find the exit status entry for this task in the parent TCB */

      child = group_find_child(group, pid);
      if (child)
        {
          /* Mark that all members of the child task group has exited */

          child->ch_flags |= CHILD_FLAG_EXITED;
        }
    }
}

#else

#  define nxtask_groupexit(group,pid)

#endif /* CONFIG_SCHED_CHILD_STATUS */

/****************************************************************************
 * Name: nxtask_reportexitstatus
 *
 * Description:
 *   Record the exit status of a child task in its parent task group.
 *
 * Input Parameters:
 *   ppid   - The ID of the parent task group to notify.
 *   ctcb   - The TCB of the exiting task.
 *   status - The exit status of the exiting task.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
#ifdef HAVE_GROUP_MEMBERS
static inline void nxtask_reportexitstatus(pid_t ppid,
                                        FAR struct tcb_s *ctcb,
                                        int status)
{
  FAR struct task_group_s *chgrp = ctcb->group;
  FAR struct task_group_s *pgrp;

  DEBUGASSERT(chgrp);

  /* Get the parent task group.  It is possible that all of the members of
   * the parent task group have exited.  This would not be an error.  In
   * this case, the child task group has been orphaned.
   */

  pgrp = task_getgroup(ppid);
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
      nxtask_exitstatus(pgrp, ctcb, status);
    }
}

/****************************************************************************
 * Name: nxtask_groupexit_notify
 *
 * Description:
 *   Mark the child task group as exited in the parent and send SIGCHLD.
 *   Must be called exactly once, by the member that group_leave() reported
 *   as the last one to leave the group.
 *
 * Input Parameters:
 *   ppid  - The ID of the parent task group to notify.
 *   chpid - The ID of the child task that exited.
 *   chuid - The user identity of the child task, when
 *           CONFIG_SCHED_USER_IDENTITY is enabled; 0 otherwise.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxtask_groupexit_notify(pid_t ppid, pid_t chpid, uid_t chuid)
{
  FAR struct task_group_s *pgrp;
  irqstate_t flags;
  siginfo_t info;

  /* The last-member decision must be made atomically with the removal
   * from tg_members (in group_leave()).  Deciding it earlier with
   * sq_is_singular() races: two exiting members can each see the other
   * still present, and SIGCHLD is then never sent.
   */

  flags = enter_critical_section();
  sched_lock();

  pgrp = task_getgroup(ppid);
  if (pgrp != NULL)
    {
      /* Mark that all of the threads in the task group have exited */

      nxtask_groupexit(pgrp, chpid);

      /* Create the siginfo structure.  We don't actually know the cause.
       * That is a bug. Let's just say that the child task just exited
       * for now.
       */

      memset(&info, 0, sizeof(info));
      info.si_signo           = SIGCHLD;
      info.si_code            = CLD_EXITED;
      info.si_errno           = OK;
      info.si_value.sival_ptr = NULL;
      info.si_pid             = chpid;
#ifdef CONFIG_SCHED_USER_IDENTITY
      info.si_uid             = chuid;
#else
      info.si_uid             = 0;
#endif
      info.si_status          = pgrp->tg_exitcode;

      /* Send the signal to one thread in the group */

      group_signal(pgrp, &info, NULL);
    }

  sched_unlock();
  leave_critical_section(flags);
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

      nxtask_exitstatus(ptcb->group, ctcb, status);

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
#ifdef CONFIG_SCHED_USER_IDENTITY
      info.si_uid             = ctcb->group->tg_uid;
#else
      info.si_uid             = 0;
#endif
      info.si_status          = status;

      /* Send the signal.  We need to use this internal interface so that we
       * can provide the correct si_code value with the signal.
       */

      nxsig_tcbdispatch(ptcb, &info, true, NULL);
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

  /* Send SIGCHLD to all members of the parent's task group */

  nxtask_reportexitstatus(ctcb->group->tg_ppid, ctcb, status);
#else
  FAR struct tcb_s *ptcb;

  /* Get the TCB of the receiving, parent task.  We do this early to
   * handle multiple calls to nxtask_signalparent.
   */

  ptcb = nxsched_get_tcb(ctcb->group->tg_ppid);
  if (ptcb == NULL)
    {
      /* The parent no longer exists... bail */

      return;
    }

  /* Send SIGCHLD to all members of the parent's task group.  NOTE that the
   * SIGCHLD signal is only sent once either (1) if this is the final thread
   * of the task group that is exiting (HAVE_GROUP_MEMBERS) or (2) if the
   * main thread of the group is exiting (!HAVE_GROUP_MEMBERS).
   */

  nxtask_sigchild(ptcb, ctcb, status);
#endif
}
#else
#  define nxtask_signalparent(ctcb,status)
#endif

/****************************************************************************
 * Name: nxtask_saveexitstatus
 *
 * Description:
 *   Record the exit status for a task waiting in waitpid()
 *
 * Input Parameters:
 *   tcb    - The TCB of the exiting task.
 *   status - The exit status of the exiting task.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
static inline void nxtask_saveexitstatus(FAR struct tcb_s *tcb, int status)
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
          if (group->tg_statloc != NULL)
            {
              *group->tg_statloc = status << 8;
            }
        }
    }
}

/****************************************************************************
 * Name: nxtask_exitwakeup
 *
 * Description:
 *   Wakeup any tasks waiting for this task group to exit
 *
 * Input Parameters:
 *   group - The task group that the last member just left.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nxtask_exitwakeup(FAR struct task_group_s *group)
{
  int semvalue;

  /* Wakeup any tasks waiting for this task to exit */

  group->tg_statloc   = NULL;
  group->tg_waitflags = 0;

  nxsem_get_value(&group->tg_exitsem, &semvalue);
  while (semvalue < 0)
    {
      /* Wake up the thread */

      nxsem_post(&group->tg_exitsem);
      nxsem_get_value(&group->tg_exitsem, &semvalue);
    }
}
#else
#  define nxtask_saveexitstatus(tcb, status)
#  define nxtask_exitwakeup(group)
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
#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(HAVE_GROUP_MEMBERS)
  pid_t ppid = INVALID_PROCESS_ID;
  pid_t chpid = INVALID_PROCESS_ID;
  uid_t chuid = 0;
#elif defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  FAR struct task_group_s *group;
#endif
#ifdef CONFIG_SCHED_DUMP_LEAK
  struct mm_memdump_s dump =
  {
    tcb->pid,
#  if CONFIG_MM_BACKTRACE >= 0
    0,
    ULONG_MAX
#  endif
  };
#endif

  /* Under certain conditions, nxtask_exithook() can be called multiple
   * times.  A bit in the TCB was set the first time this function was
   * called.  If that bit is set, then just exit doing nothing more..
   */

  DEBUGASSERT((tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0);

  nxsched_dumponexit();

  /* If the task was terminated by another task, it may be in an unknown
   * state.  Make some feeble effort to recover the state.
   */

  nxtask_recover(tcb);

  /* Disable the scheduling function to prevent other tasks from
   * being deleted after they are awakened
   */

  sched_lock();

  /* Send the SIGCHLD signal to the parent task group */

  nxtask_signalparent(tcb, status);

  /* Record the exit status for waitpid(); the waiters are woken only
   * after the group has been left and its files closed.
   */

  nxtask_saveexitstatus(tcb, status);

  sched_unlock();

#ifndef CONFIG_DISABLE_ALL_SIGNALS
  /* Release the signals that wait for this thread in the pending list of
   * the group.  No new ones come: the thread is in exit processing.
   */

  nxsig_discard_threadsignals(tcb);
#endif

  /* Leave the task group.  Perhaps discarding any un-reaped child
   * status (no zombies here!).  group_leave() decides "last" under
   * tg_lock, so exactly one exiting member notifies the parent, and
   * only after the files are closed.  It also detaches the group from
   * the TCB and may release it, so keep the IDs that the notification
   * needs.
   */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(HAVE_GROUP_MEMBERS)
  if (tcb->group != NULL)
    {
      ppid  = tcb->group->tg_ppid;
      chpid = tcb->group->tg_pid;
#ifdef CONFIG_SCHED_USER_IDENTITY
      chuid = tcb->group->tg_uid;
#else
      chuid = 0;
#endif
    }

  if (group_leave(tcb))
    {
      nxtask_groupexit_notify(ppid, chpid, chuid);
    }
#elif defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  group = tcb->group;
  if (group != NULL)
    {
      /* Hold the group: group_leave() releases it when this is the last
       * member and nobody else waits for the exit status.
       */

      group_add_waiter(group);
    }

  if (group_leave(tcb))
    {
      nxtask_exitwakeup(group);
    }

  if (group != NULL)
    {
      group_del_waiter(group);
    }
#else
  group_leave(tcb);
#endif

  /* Deallocate anything left in the TCB's queues */

#ifndef CONFIG_DISABLE_ALL_SIGNALS
  nxsig_cleanup(tcb); /* Deallocate Signal lists */
#endif

#ifdef CONFIG_SCHED_DUMP_LEAK
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      kmm_memdump(&dump);
    }
  else
    {
      umm_memdump(&dump);
    }
#endif
}
