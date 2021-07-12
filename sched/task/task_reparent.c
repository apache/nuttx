/****************************************************************************
 * sched/task/task_reparent.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>

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
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  pid_t opid;
  int ret;

  /* Disable interrupts so that nothing can change in the relationship of
   * the three task:  Child, current parent, and new parent.
   */

  flags = enter_critical_section();

  /* Get the child tasks task group */

  tcb = nxsched_get_tcb(chpid);
  if (!tcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  DEBUGASSERT(tcb->group);
  chgrp = tcb->group;

  /* Get the PID of the old parent task's task group (opid) */

  opid = chgrp->tg_ppid;

  /* Get the old parent task's task group (ogrp) */

  ogrp = group_findbypid(opid);
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

      ppid = ogrp->tg_ppid;
      pgrp = group_findbypid(ppid);
    }
  else
    {
      /* Get the new parent task's task group (pgrp) */

      tcb = nxsched_get_tcb(ppid);
      if (!tcb)
        {
          ret = -ESRCH;
          goto errout_with_ints;
        }

      pgrp = tcb->group;
      ppid = pgrp->tg_pid;
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

  chgrp->tg_ppid = ppid;

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Remove the child status entry from old parent task group */

  child = group_remove_child(ogrp, chpid);
  if (child)
    {
      /* Has the new parent's task group suppressed child exit status? */

      if ((pgrp->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
        {
          /* No.. Add the child status entry to the new parent's task group */

          group_add_child(pgrp, child);
        }
      else
        {
          /* Yes.. Discard the child status entry */

          group_free_child(child);
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

  ogrp->tg_nchildren--;  /* The original parent now has one few children */
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
  FAR struct tcb_s *ptcb;
  FAR struct tcb_s *chtcb;
  FAR struct tcb_s *otcb;
  pid_t opid;
  irqstate_t flags;
  int ret;

  /* Disable interrupts so that nothing can change in the relationship of
   * the three task:  Child, current parent, and new parent.
   */

  flags = enter_critical_section();

  /* Get the child tasks TCB (chtcb) */

  chtcb = nxsched_get_tcb(chpid);
  if (!chtcb)
    {
      ret = -ECHILD;
      goto errout_with_ints;
    }

  /* Get the PID of the child task's parent (opid) */

  opid = chtcb->group->tg_ppid;

  /* Get the TCB of the child task's parent (otcb) */

  otcb = nxsched_get_tcb(opid);
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

  ptcb = nxsched_get_tcb(ppid);
  if (!ptcb)
    {
      ret = -ESRCH;
      goto errout_with_ints;
    }

  /* Then reparent the child.  The task specified by ppid is the new
   * parent.
   */

  chtcb->group->tg_ppid = ppid;

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Remove the child status entry from old parent TCB */

  child = group_remove_child(otcb->group, chpid);
  if (child)
    {
      /* Has the new parent's task group suppressed child exit status? */

      if ((ptcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
        {
          /* No.. Add the child status entry to the new parent's task group */

          group_add_child(ptcb->group, child);
        }
      else
        {
          /* Yes.. Discard the child status entry */

          group_free_child(child);
        }

      /* Either case is a success */

      ret = OK;
    }
  else
    {
      /* This would not be an error if the original parent's task group has
       * suppressed child exit status.
       */

      ret = ((otcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0) ?
              -ENOENT : OK;
    }

#else /* CONFIG_SCHED_CHILD_STATUS */
  /* Child task exit status is not retained */

  DEBUGASSERT(otcb->group != NULL && otcb->group->tg_nchildren > 0);

  otcb->group->tg_nchildren--;  /* The original parent now has one few children */
  ptcb->group->tg_nchildren++;  /* The new parent has one additional child */
  ret = OK;

#endif /* CONFIG_SCHED_CHILD_STATUS */

errout_with_ints:
  leave_critical_section(flags);
  return ret;
}
#endif
#endif /* CONFIG_SCHED_HAVE_PARENT */
