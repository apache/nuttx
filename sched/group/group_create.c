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

#include <string.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
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
  group->tg_uid  = rgroup->tg_uid;
  group->tg_gid  = rgroup->tg_gid;
  group->tg_euid = rgroup->tg_euid;
  group->tg_egid = rgroup->tg_egid;
  group->tg_suid = rgroup->tg_suid;
  group->tg_sgid = rgroup->tg_sgid;
#if CONFIG_SCHED_NGROUPS > 0
  group->tg_ngroups = rgroup->tg_ngroups;
  if (rgroup->tg_ngroups > 0)
    {
      memcpy(group->tg_groups, rgroup->tg_groups,
             rgroup->tg_ngroups * sizeof(gid_t));
    }
#endif
}
#else
#  define group_inherit_identity(group)
#endif

#ifdef CONFIG_FS_CHROOT
/****************************************************************************
 * Name: group_inherit_chroot
 *
 * Description:
 *   Inherit the chroot jail from the parent task group.
 *
 ****************************************************************************/

static int group_inherit_chroot(FAR struct task_group_s *group)
{
  FAR struct tcb_s *rtcb          = this_task();
  FAR struct task_group_s *rgroup = rtcb->group;

  DEBUGASSERT(group != NULL && rgroup != NULL);

  if (rgroup->tg_root == NULL)
    {
      return OK;
    }

  group->tg_root = rgroup->tg_root;
  inode_addref(group->tg_root);

  if (rgroup->tg_rootrel != NULL)
    {
      size_t len = strlen(rgroup->tg_rootrel) + 1;

      group->tg_rootrel = kmm_malloc(len);
      if (group->tg_rootrel == NULL)
        {
          inode_release(group->tg_root);
          group->tg_root = NULL;
          return -ENOMEM;
        }

      memcpy(group->tg_rootrel, rgroup->tg_rootrel, len);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_allocate
 *
 * Description:
 *   Create and a new task group structure for the specified TCB. This
 *   function is called as part of the task creation sequence.  The structure
 *   allocated and zeroed, but otherwise uninitialized.  The full creation
 *   of the group of a two step process:  (1) First, this function allocates
 *   group structure early in the task creation sequence in order to provide
 *   a group container, then (2) group_initialize() is called to set up the
 *   group membership.
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

int group_allocate(FAR struct tcb_s *tcb, uint8_t ttype)
{
  FAR struct task_group_s *group;
  int ret;

  DEBUGASSERT(tcb && !tcb->group);

  ttype &= TCB_FLAG_TTYPE_MASK;

  /* Initialize group pointer and assign to TCB */

  if (ttype == TCB_FLAG_TTYPE_KERNEL)
    {
      group = &g_kthread_group;
      tcb->group = group;
      if (group->tg_info)
        {
          return OK;
        }
    }
  else
    {
      group = kmm_zalloc(sizeof(struct task_group_s));
    }

  if (!group)
    {
      return -ENOMEM;
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

#ifdef CONFIG_FS_BACKTRACE_DEFAULT
  /* Enable FD backtrace for the group by default */

  group->tg_flags |= GROUP_FLAG_FD_BACKTRACE;
#endif

  group->tg_flags |= GROUP_FLAG_DUMPABLE;

  /* Attach the group to the TCB */

  tcb->group = group;

  /* Inherit the user identity from the parent task group */

  group_inherit_identity(group);

#ifdef CONFIG_FS_CHROOT
  /* Kernel threads share g_kthread_group and must not inherit a user jail */

  if (ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      ret = group_inherit_chroot(group);
      if (ret < 0)
        {
          goto errout_with_group;
        }
    }
#endif

  /* Initialize file descriptors for the TCB */

  fdlist_init(&group->tg_fdlist);

  /* Alloc task info for group  */

  ret = task_init_info(group);
  if (ret < 0)
    {
      goto errout_with_group;
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

errout_with_group:
#ifdef CONFIG_FS_CHROOT
  if (group->tg_root != NULL)
    {
      inode_release(group->tg_root);
      group->tg_root = NULL;
    }

  if (group->tg_rootrel != NULL)
    {
      kmm_free(group->tg_rootrel);
      group->tg_rootrel = NULL;
    }
#endif

  kmm_free(group);
  return ret;
}

/****************************************************************************
 * Name: group_initialize
 *
 * Description:
 *   Add the task as the initial member of the group.  The full creation of
 *   the group of a two step process:  (1) First, this group structure is
 *   allocated by group_allocate() early in the task creation sequence, then
 *   (2) this function  is called to set up the initial group membership.
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

void group_initialize(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;
  spin_lock_init(&group->tg_lock);

  /* Allocate mm_map list if required */

  mm_map_initialize(&group->tg_mm_map,
                    (tcb->flags & TCB_FLAG_TTYPE_KERNEL) != 0);

#ifdef HAVE_GROUP_MEMBERS
  /* Assign the PID of this new task as a member of the group. */

  sq_addlast(&tcb->member, &group->tg_members);
#endif

  /* Save the ID of the main task within the group of threads.  This needed
   * for things like SIGCHLD.  It ID is also saved in the TCB of the main
   * task but is also retained in the group which may persist after the main
   * task has exited.
   */

  if (group != &g_kthread_group)
    {
      group->tg_pid = tcb->pid;
    }

  group->tg_info->ta_pid = group->tg_pid;
}
