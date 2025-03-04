/****************************************************************************
 * sched/group/group_leave.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>

#ifdef CONFIG_BINFMT_LOADABLE
#  include <nuttx/binfmt/binfmt.h>
#endif

#include "environ/environ.h"
#include "signal/signal.h"
#include "pthread/pthread.h"
#include "mqueue/mqueue.h"
#include "group/group.h"
#include "tls/tls.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_release
 *
 * Description:
 *   Release group resources after the last member has left the group.
 *
 * Input Parameters:
 *   group - The group to be removed.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

static inline void
group_release(FAR struct task_group_s *group, uint8_t ttype)
{
  /* Destroy the mutex */

  nxrmutex_destroy(&group->tg_mutex);

  task_uninit_info(group);

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  /* Free all un-reaped child exit status */

  group_remove_children(group);
#endif

  /* Release pending signals */

  nxsig_release(group);

#ifndef CONFIG_DISABLE_PTHREAD
  /* Release pthread resources */

  pthread_release(group);
#endif

  /* Free all file-related resources now.  We really need to close files as
   * soon as possible while we still have a functioning task.
   */

  /* Free resources held by the file descriptor list */

  files_putlist(&group->tg_filelist);

#ifndef CONFIG_DISABLE_ENVIRON
  /* Release all shared environment variables */

  env_release(group);
#endif

  /* Destroy the mm_map list */

  mm_map_destroy(&group->tg_mm_map);

#ifdef CONFIG_BINFMT_LOADABLE
  /* If the exiting task was loaded into RAM from a file, then we need to
   * lease all of the memory resource when the last thread exits the task
   * group.
   */

  if (group->tg_bininfo != NULL)
    {
      binfmt_exit(group->tg_bininfo);
      group->tg_bininfo = NULL;
    }
#endif

  /* Then drop the group freeing the allocated memory */

#ifndef CONFIG_DISABLE_PTHREAD
  if (ttype == TCB_FLAG_TTYPE_PTHREAD)
    {
      /* Mark the group as deleted now */

      group->tg_flags |= GROUP_FLAG_DELETED;

      group_drop(group);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_leave
 *
 * Description:
 *   Release a reference on a group.  This function is called when a task or
 *   thread exits.  It decrements the reference count on the group.  If the
 *   reference count decrements to zero, then it frees the group and all of
 *   resources contained in the group.
 *
 * Input Parameters:
 *   tcb - The TCB of the task that is exiting.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

void group_leave(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;
#ifdef HAVE_GROUP_MEMBERS
  irqstate_t flags;
#endif

  DEBUGASSERT(tcb);

  /* Make sure that we have a group. */

  group = tcb->group;
  if (group)
    {
      /* In any event, we can detach the group from the TCB so that we won't
       * do this again.
       */

      tcb->group = NULL;

      /* Remove the member from group. */

#ifdef HAVE_GROUP_MEMBERS
      flags = spin_lock_irqsave(&group->tg_lock);
      sq_rem(&tcb->member, &group->tg_members);
      spin_unlock_irqrestore(&group->tg_lock, flags);

      /* Have all of the members left the group? */

      if (sq_empty(&group->tg_members))
#endif
        {
          /* Yes.. Release all of the resource held by the task group */

          group_release(group, tcb->flags & TCB_FLAG_TTYPE_MASK);
        }
    }
}

/****************************************************************************
 * Name: group_drop
 *
 * Description:
 *   Release the group's memory. This function is called whenever a reference
 *   to the group structure is released. It is not dependent on member count,
 *   but rather external references, which include:
 *   - Waiter list for waitpid()
 *
 * Input Parameters:
 *   group - The group that is to be dropped
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task deletion or context switch in a safe context.  No
 *   special precautions are required here.
 *
 ****************************************************************************/

void group_drop(FAR struct task_group_s *group)
{
  FAR struct task_tcb_s *tcb;

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* If there are threads waiting for this group to be freed, then we cannot
   * yet free the memory resources.  Instead just mark the group deleted
   * and wait for those threads complete their waits.
   */

  if (group->tg_nwaiters > 0)
    {
      /* Hold the group still */

      sinfo("Keep group %p (waiters > 0)\n", group);
    }
  else
#endif
  /* Finally, if no one needs the group and it has been deleted, remove it */

  if (group->tg_flags & GROUP_FLAG_DELETED)
    {
      tcb = container_of(group, struct task_tcb_s, group);

      /* Release the group container itself */

      if (tcb->cmn.flags & TCB_FLAG_FREE_TCB)
        {
          kmm_free(tcb);
        }
    }
}
