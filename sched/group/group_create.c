/****************************************************************************
 * sched/group/group_create.c
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

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "tls/tls.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct task_group_s  g_kthread_group;   /* Shared among kthreads     */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_inherit_identity
 *
 * Description:
 *   All inherit the user identity from the parent task group.
 *
 * Input Parameters:
 *   group - The new task group.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The parent of the new task is the task at the head of the assigned task
 *   list for the current CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_USER_IDENTITY
static inline void group_inherit_identity(FAR struct task_group_s *group)
{
  FAR struct tcb_s *rtcb          = this_task();
  FAR struct task_group_s *rgroup = rtcb->group;

  /* Inherit the user identity from the parent task group. */

  DEBUGASSERT(group != NULL);
  group->tg_uid = rgroup->tg_uid;
  group->tg_gid = rgroup->tg_gid;
  group->tg_euid = rgroup->tg_euid;
  group->tg_egid = rgroup->tg_egid;
}
#else
#  define group_inherit_identity(group)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_initialize
 *
 * Description:
 *   Create and a new task group structure for the specified TCB. This
 *   function is called as part of the task creation sequence.  The structure
 *   allocated and zeroed, but otherwise uninitialized.  The full creation
 *   of the group of a two step process:  (1) First, this function allocates
 *   group structure early in the task creation sequence in order to provide
 *   a group container, then (2) group_postinitialize() is called to set up
 *   the group membership.
 *
 * Input Parameters:
 *   tcb   - The tcb in need of the task group.
 *   ttype - Type of the thread that is the parent of the group
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

int group_initialize(FAR struct task_tcb_s *tcb, uint8_t ttype)
{
  FAR struct task_group_s *group;
  int ret;

  DEBUGASSERT(tcb && !tcb->cmn.group);

  ttype &= TCB_FLAG_TTYPE_MASK;

  /* Initialize group pointer and assign to TCB */

  if (ttype == TCB_FLAG_TTYPE_KERNEL)
    {
      group = &g_kthread_group;
      tcb->cmn.group = group;
      if (group->tg_info)
        {
          return OK;
        }
    }
  else
    {
      group = &tcb->group;
    }

#if defined(CONFIG_MM_KERNEL_HEAP)
  /* If this group is being created for a privileged thread, then all
   * elements of the group must be created for privileged access.
   */

  if ((ttype & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      group->tg_flags |= GROUP_FLAG_PRIVILEGED;
    }
#endif /* defined(CONFIG_MM_KERNEL_HEAP) */

#ifdef HAVE_GROUP_MEMBERS
  /* Initialize member list of the group */

  sq_init(&group->tg_members);
#endif

  /* Attach the group to the TCB */

  tcb->cmn.group = group;

  /* Inherit the user identity from the parent task group */

  group_inherit_identity(group);

  /* Initialize file descriptors for the TCB */

  files_initlist(&group->tg_filelist);

  /* Alloc task info for group  */

  ret = task_init_info(group);
  if (ret < 0)
    {
      return ret;
    }

  nxrmutex_init(&group->tg_mutex);

#ifndef CONFIG_DISABLE_PTHREAD
  /* Initialize the task group join */

  sq_init(&group->tg_joinqueue);
#endif

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* Initialize the exit/wait semaphores */

  nxsem_init(&group->tg_exitsem, 0, 0);
#endif

  return OK;
}

/****************************************************************************
 * Name: group_postinitialize
 *
 * Description:
 *   Add the task as the initial member of the group.  The full creation of
 *   the group of a two step process:  (1) First, this group structure is
 *   allocated by group_initialize() early in the task creation sequence,
 *   then (2) this function  is called to set up the initial group
 *   membership.
 *
 * Input Parameters:
 *   tcb - The tcb in need of the task group.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

void group_postinitialize(FAR struct task_tcb_s *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb && tcb->cmn.group);
  group = tcb->cmn.group;
  spin_lock_init(&group->tg_lock);

  /* Allocate mm_map list if required */

  mm_map_initialize(&group->tg_mm_map,
                    (tcb->cmn.flags & TCB_FLAG_TTYPE_KERNEL) != 0);

#ifdef HAVE_GROUP_MEMBERS
  /* Assign the PID of this new task as a member of the group. */

  sq_addlast(&tcb->cmn.member, &group->tg_members);
#endif

  /* Save the ID of the main task within the group of threads.  This needed
   * for things like SIGCHLD.  It ID is also saved in the TCB of the main
   * task but is also retained in the group which may persist after the main
   * task has exited.
   */

  if (group != &g_kthread_group)
    {
      group->tg_pid = tcb->cmn.pid;
    }
}
