/****************************************************************************
 * sched/group/group_create.c
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
#include <nuttx/lib/lib.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "tls/tls.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is this worth making a configuration option? */

#define GROUP_INITIAL_MEMBERS 4

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
/* This is the head of a list of all group members */

FAR struct task_group_s *g_grouphead;
#endif

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
}
#else
#  define group_inherit_identity(group)
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

int group_allocate(FAR struct task_tcb_s *tcb, uint8_t ttype)
{
  FAR struct task_group_s *group;
  int ret = -ENOMEM;

  DEBUGASSERT(tcb && !tcb->cmn.group);

  /* Allocate the group structure and assign it to the TCB */

  group = (FAR struct task_group_s *)kmm_zalloc(sizeof(struct task_group_s));
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

# if defined(CONFIG_FILE_STREAM)
  /* In a flat, single-heap build.  The stream list is allocated with the
   * group structure.  But in a kernel build with a kernel allocator, it
   * must be separately allocated using a user-space allocator.
   *
   * REVISIT:  Kernel threads should not require a stream allocation.  They
   * should not be using C buffered I/O at all.
   */

  group->tg_streamlist = (FAR struct streamlist *)
    group_zalloc(group, sizeof(struct streamlist));
  if (!group->tg_streamlist)
    {
      goto errout_with_group;
    }

# endif /* defined(CONFIG_FILE_STREAM) */
#endif /* defined(CONFIG_MM_KERNEL_HEAP) */

#ifdef HAVE_GROUP_MEMBERS
  /* Allocate space to hold GROUP_INITIAL_MEMBERS members of the group */

  group->tg_members = kmm_malloc(GROUP_INITIAL_MEMBERS * sizeof(pid_t));
  if (!group->tg_members)
    {
      goto errout_with_stream;
    }

  /* Number of members in allocation */

  group->tg_mxmembers = GROUP_INITIAL_MEMBERS;
#endif

  /* Alloc task info for group  */

  ret = task_init_info(group);
  if (ret < 0)
    {
      goto errout_with_member;
    }

  /* Attach the group to the TCB */

  tcb->cmn.group = group;

  /* Inherit the user identity from the parent task group */

  group_inherit_identity(group);

  /* Initialize file descriptors for the TCB */

  files_initlist(&group->tg_filelist);

#ifdef CONFIG_FILE_STREAM
  /* Initialize file streams for the task group */

  lib_stream_initialize(group);
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Initialize the pthread join mutex */

  nxmutex_init(&group->tg_joinlock);
#endif

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* Initialize the exit/wait semaphores
   *
   * This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&group->tg_exitsem, 0, 0);
  nxsem_set_protocol(&group->tg_exitsem, SEM_PRIO_NONE);
#endif

  return OK;

errout_with_member:
#ifdef HAVE_GROUP_MEMBERS
  kmm_free(group->tg_members);
errout_with_stream:
#endif
#if defined(CONFIG_FILE_STREAM) && defined(CONFIG_MM_KERNEL_HEAP)
  group_free(group, group->tg_streamlist);
errout_with_group:
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

void group_initialize(FAR struct task_tcb_s *tcb)
{
  FAR struct task_group_s *group;
#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  irqstate_t flags;
#endif

  DEBUGASSERT(tcb && tcb->cmn.group);
  group = tcb->cmn.group;

#ifdef HAVE_GROUP_MEMBERS
  /* Assign the PID of this new task as a member of the group. */

  group->tg_members[0] = tcb->cmn.pid;
#endif

  /* Save the ID of the main task within the group of threads.  This needed
   * for things like SIGCHLD.  It ID is also saved in the TCB of the main
   * task but is also retained in the group which may persist after the main
   * task has exited.
   */

  group->tg_pid = tcb->cmn.pid;

  /* Mark that there is one member in the group, the main task */

  group->tg_nmembers = 1;

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  /* Add the initialized entry to the list of groups */

  flags = enter_critical_section();
  group->flink = g_grouphead;
  g_grouphead = group;
  leave_critical_section(flags);
#endif
}
