/****************************************************************************
 * sched/task/task_reparent.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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

#include <errno.h>

#include <nuttx/irq.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

#ifdef CONFIG_SCHED_HAVE_PARENT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_reparent
 *
 * Description:
 *   Change the parent of a task.
 *
 * Input Parameters:
 *   ppid - PID of the new parent task (0 for grandparent, i.e. the parent
 *     of the current parent task)
 *   chpid  - PID of the child to be reparented.
 *
 * Returned Value:
 *   0 (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
int task_reparent(pid_t ppid, pid_t chpid)
{
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
#endif
  FAR struct task_group_s *chgrp;
  FAR struct task_group_s *ogrp;
  FAR struct task_group_s *pgrp;
  struct tcb_s *tcb;
  gid_t ogid;
  gid_t pgid;
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that nothing can change in the relationship of
   * the three task:  Child, current parent, and new parent.
   */

  flags = enter_critical_section();

  /* Get the child tasks task group */

  tcb = sched_gettcb(chpid);
  if (!tcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  DEBUGASSERT(tcb->group);
  chgrp = tcb->group;

  /* Get the GID of the old parent task's task group (ogid) */

  ogid = chgrp->tg_pgid;

  /* Get the old parent task's task group (ogrp) */

  ogrp = group_findbygid(ogid);
  if (!ogrp)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* If new parent task's PID (ppid) is zero, then new parent is the
   * grandparent will be the new parent, i.e., the parent of the current
   * parent task.
   */

  if (ppid == 0)
    {
      /* Get the grandparent task's task group (pgrp) */

      pgid = ogrp->tg_pgid;
      pgrp = group_findbygid(pgid);
    }
  else
    {
      /* Get the new parent task's task group (pgrp) */

      tcb = sched_gettcb(ppid);
      if (!tcb)
        {
          ret = -ESRCH;
          goto errout_with_ints;
        }

      pgrp = tcb->group;
      pgid = pgrp->tg_gid;
    }

  if (!pgrp)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* Then reparent the child.  Notice that we don't actually change the
   * parent of the task. Rather, we change the parent task group for
   * all members of the child's task group.
   */

  chgrp->tg_pgid = pgid;

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Remove the child status entry from old parent task group */

  child = group_removechild(ogrp, chpid);
  if (child)
    {
      /* Has the new parent's task group supressed child exit status? */

      if ((pgrp->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
        {
          /* No.. Add the child status entry to the new parent's task group */

          group_addchild(pgrp, child);
        }
      else
        {
          /* Yes.. Discard the child status entry */

          group_freechild(child);
        }

      /* Either case is a success */

      ret = OK;
    }
  else
    {
      /* This would not be an error if the original parent's task group has
       * suppressed child exit status.
       */

      ret = ((ogrp->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0) ? -ENOENT : OK;
    }

#else /* CONFIG_SCHED_CHILD_STATUS */
  /* Child task exit status is not retained */

  DEBUGASSERT(ogrp->tg_nchildren > 0);

  ogrp->tg_nchildren--;  /* The orignal parent now has one few children */
  pgrp->tg_nchildren++;  /* The new parent has one additional child */
  ret = OK;

#endif /* CONFIG_SCHED_CHILD_STATUS */

errout_with_ints:
  leave_critical_section(flags);
  return ret;
}
#else
int task_reparent(pid_t ppid, pid_t chpid)
{
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
#endif
  struct tcb_s *ptcb;
  struct tcb_s *chtcb;
  struct tcb_s *otcb;
  pid_t opid;
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that nothing can change in the relationship of
   * the three task:  Child, current parent, and new parent.
   */

  flags = enter_critical_section();

  /* Get the child tasks TCB (chtcb) */

  chtcb = sched_gettcb(chpid);
  if (!chtcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  /* Get the PID of the child task's parent (opid) */

  opid = chtcb->group->tg_ppid;

  /* Get the TCB of the child task's parent (otcb) */

  otcb = sched_gettcb(opid);
  if (!otcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* If new parent task's PID (tg_ppid) is zero, then new parent is the
   * grandparent will be the new parent, i.e., the parent of the current
   * parent task.
   */

  if (ppid == 0)
    {
      ppid = otcb->group->tg_ppid;
    }

  /* Get the new parent task's TCB (ptcb) */

  ptcb = sched_gettcb(ppid);
  if (!ptcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* Then reparent the child.  The task specified by ppid is the new parent. */

  chtcb->group->tg_ppid = ppid;

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Remove the child status entry from old parent TCB */

  child = group_removechild(otcb->group, chpid);
  if (child)
    {
      /* Has the new parent's task group supressed child exit status? */

      if ((ptcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
        {
          /* No.. Add the child status entry to the new parent's task group */

          group_addchild(ptcb->group, child);
        }
      else
        {
          /* Yes.. Discard the child status entry */

          group_freechild(child);
        }

      /* Either case is a success */

      ret = OK;
    }
  else
    {
      /* This would not be an error if the original parent's task group has
       * suppressed child exit status.
       */

      ret = ((otcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0) ? -ENOENT : OK;
    }

#else /* CONFIG_SCHED_CHILD_STATUS */
  /* Child task exit status is not retained */

  DEBUGASSERT(otcb->group != NULL && otcb->group->tg_nchildren > 0);

  otcb->group->tg_nchildren--;  /* The orignal parent now has one few children */
  ptcb->group->tg_nchildren++;  /* The new parent has one additional child */
  ret = OK;

#endif /* CONFIG_SCHED_CHILD_STATUS */

errout_with_ints:
  leave_critical_section(flags);
  return ret;
}
#endif
#endif /* CONFIG_SCHED_HAVE_PARENT */
