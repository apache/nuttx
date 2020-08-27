/****************************************************************************
 *  sched/group/group_create.c
 *
 *   Copyright (C) 2013, 2016, 2019 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>

#include "environ/environ.h"
#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is this worth making a configuration option? */

#define GROUP_INITIAL_MEMBERS 4

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is counter that is used to generate unique task group IDs */

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
static grpid_t g_grpid_counter;
#endif

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
 * Name: group_assign_grpid
 *
 * Description:
 *   Create a unique group ID.
 *
 * Input Parameters:
 *   tcb - The tcb in need of the task group.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
static void group_assign_grpid(FAR struct task_group_s *group)
{
  irqstate_t flags;
  grpid_t grpid;

  /* Pre-emption should already be disabled, but let's be paranoid careful */

  sched_lock();

  /* Loop until we create a unique ID */

  for (; ; )
    {
      /* Increment the ID counter. It is global data so be extra paranoid. */

      flags = enter_critical_section();
      grpid = ++g_grpid_counter;

      /* Check for overflow */

      if (grpid <= 0)
        {
          g_grpid_counter = 1;            /* One is the IDLE group */
          leave_critical_section(flags);
        }
      else
        {
          /* Does a task group with this ID already exist? */

          leave_critical_section(flags);
          if (group_findby_grpid(grpid) == NULL)
            {
              /* No.. Assign this ID to the new group and return */

              group->tg_grpid = grpid;
              sched_unlock();
              return;
            }
        }
    }
}
#endif /* HAVE_GROUP_MEMBERS */

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
  int ret;

  DEBUGASSERT(tcb && !tcb->cmn.group);

  /* Allocate the group structure and assign it to the TCB */

  group = (FAR struct task_group_s *)kmm_zalloc(sizeof(struct task_group_s));
  if (!group)
    {
      return -ENOMEM;
    }

#if defined(CONFIG_FILE_STREAM) && defined(CONFIG_MM_KERNEL_HEAP)
  /* If this group is being created for a privileged thread, then all
   * elements of the group must be created for privileged access.
   */

  if ((ttype & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      group->tg_flags |= GROUP_FLAG_PRIVILEGED;
    }

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
      kmm_free(group);
      return -ENOMEM;
    }

#endif

  /* Attach the group to the TCB */

  tcb->cmn.group = group;

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  /* Assign the group a unique ID.  If g_grpid_counter were to wrap before we
   * finish with task creation, that would be a problem.
   */

  group_assign_grpid(group);
#endif

  /* Inherit the user identity from the parent task group */

  group_inherit_identity(group);

  /* Duplicate the parent tasks environment */

  ret = env_dup(group);
  if (ret < 0)
    {
#if defined(CONFIG_FILE_STREAM) && defined(CONFIG_MM_KERNEL_HEAP)
      group_free(group, group->tg_streamlist);
#endif
      kmm_free(group);
      tcb->cmn.group = NULL;
      return ret;
    }

#ifndef CONFIG_DISABLE_PTHREAD
  /* Initialize the pthread join semaphore */

  nxsem_init(&group->tg_joinsem, 0, 1);
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
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

int group_initialize(FAR struct task_tcb_s *tcb)
{
  FAR struct task_group_s *group;
#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  irqstate_t flags;
#endif

  DEBUGASSERT(tcb && tcb->cmn.group);
  group = tcb->cmn.group;

#ifdef HAVE_GROUP_MEMBERS
  /* Allocate space to hold GROUP_INITIAL_MEMBERS members of the group */

  group->tg_members = kmm_malloc(GROUP_INITIAL_MEMBERS * sizeof(pid_t));
  if (!group->tg_members)
    {
      kmm_free(group);
      tcb->cmn.group = NULL;
      return -ENOMEM;
    }

  /* Assign the PID of this new task as a member of the group. */

  group->tg_members[0] = tcb->cmn.pid;

  /* Initialize the non-zero elements of group structure and assign it to
   * the tcb.
   */

  group->tg_mxmembers  = GROUP_INITIAL_MEMBERS; /* Number of members in allocation */

#endif

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  /* Add the initialized entry to the list of groups */

  flags = enter_critical_section();
  group->flink = g_grouphead;
  g_grouphead = group;
  leave_critical_section(flags);

#endif

  /* Save the ID of the main task within the group of threads.  This needed
   * for things like SIGCHILD.  It ID is also saved in the TCB of the main
   * task but is also retained in the group which may persist after the main
   * task has exited.
   */

#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_SCHED_HAVE_PARENT)
  group->tg_task = tcb->cmn.pid;
#endif

  /* Mark that there is one member in the group, the main task */

  group->tg_nmembers = 1;
  return OK;
}
