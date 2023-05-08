/****************************************************************************
 * sched/group/group_join.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "group/group.h"
#include "environ/environ.h"

#ifndef CONFIG_DISABLE_PTHREAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is this worth making a configuration option? */

#define GROUP_REALLOC_MEMBERS 4

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_addmember
 *
 * Description:
 *   Add a new member to a group.
 *
 * Input Parameters:
 *   group - The task group to add the new member
 *   pid - The new member
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during thread creation and during reparenting in a safe context.
 *   No special precautions are required here.
 *
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
static inline int group_addmember(FAR struct task_group_s *group, pid_t pid)
{
  FAR pid_t *oldmembers = NULL;
  irqstate_t flags;

  DEBUGASSERT(group && group->tg_nmembers < UINT8_MAX);

  /* Will we need to extend the size of the array of groups? */

  if (group->tg_nmembers >= group->tg_mxmembers)
    {
      FAR pid_t *newmembers;
      unsigned int newmax;

      /* Yes... reallocate the array of members */

      newmax = group->tg_mxmembers + GROUP_REALLOC_MEMBERS;
      if (newmax > UINT8_MAX)
        {
          newmax = UINT8_MAX;
        }

      newmembers = (FAR pid_t *)kmm_malloc(sizeof(pid_t) * newmax);

      if (!newmembers)
        {
          serr("ERROR: Failed to reallocate tg_members\n");
          return -ENOMEM;
        }

      /* Save the new number of members in the reallocated members array.
       * We need to make the following atomic because the member list
       * may be traversed from an interrupt handler (read-only).
       */

      flags = spin_lock_irqsave(NULL);
      memcpy(newmembers, group->tg_members,
             sizeof(pid_t) * group->tg_mxmembers);
      oldmembers = group->tg_members;
      group->tg_members   = newmembers;
      group->tg_mxmembers = newmax;
    }
  else
    {
      flags = spin_lock_irqsave(NULL);
    }

  /* Assign this new pid to the group; group->tg_nmembers will be incremented
   * by the caller.
   */

  group->tg_members[group->tg_nmembers] = pid;
  group->tg_nmembers++;
  spin_unlock_irqrestore(NULL, flags);

  if (oldmembers != NULL)
    {
      kmm_free(oldmembers);
    }

  return OK;
}
#endif /* HAVE_GROUP_MEMBERS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_bind
 *
 * Description:
 *   A thread joins the group when it is created. This is a two step process,
 *   first, the group must bound to the new threads TCB.  group_bind() does
 *   this (at the return from group_join, things are a little unstable:  The
 *   group has been bound, but tg_nmembers has not yet been incremented).
 *   Then, after the new thread is initialized and has a PID assigned to it,
 *   group_join() is called, incrementing the tg_nmembers count on the group.
 *
 * Input Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the head of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

int group_bind(FAR struct pthread_tcb_s *tcb)
{
  FAR struct tcb_s *ptcb = this_task();

  DEBUGASSERT(ptcb && tcb && ptcb->group && !tcb->cmn.group);

  /* Copy the group reference from the parent to the child */

  tcb->cmn.group = ptcb->group;
  return OK;
}

/****************************************************************************
 * Name: group_join
 *
 * Description:
 *   A thread joins the group when it is created. This is a two step process,
 *   first, the group must bound to the new threads TCB.  group_bind() does
 *   this (at the return from group_join, things are a little unstable:  The
 *   group has been bound, but tg_nmembers has not yet been incremented).
 *   Then, after the new thread is initialized and has a PID assigned to it,
 *   group_join() is called, incrementing the tg_nmembers count on the group.
 *
 * Input Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the head of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

int group_join(FAR struct pthread_tcb_s *tcb)
{
  FAR struct task_group_s *group;
#ifdef HAVE_GROUP_MEMBERS
  int ret;
#else
  irqstate_t flags;
#endif

  DEBUGASSERT(tcb && tcb->cmn.group &&
              tcb->cmn.group->tg_nmembers < UINT8_MAX);

  /* Get the group from the TCB */

  group = tcb->cmn.group;

#ifdef HAVE_GROUP_MEMBERS
  /* Add the member to the group */

  ret = group_addmember(group, tcb->cmn.pid);
  if (ret < 0)
    {
      return ret;
    }
#else
  flags = spin_lock_irqsave(NULL);
  group->tg_nmembers++;
  spin_unlock_irqrestore(NULL, flags);
#endif

  return OK;
}

#endif /* !CONFIG_DISABLE_PTHREAD */
