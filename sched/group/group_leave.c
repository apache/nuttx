/****************************************************************************
 * sched/group/group_leave.c
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
#include <nuttx/net/net.h>
#include <nuttx/lib/lib.h>
#include <nuttx/tls.h>
#include <nuttx/mm/vm_map.h>

#ifdef CONFIG_BINFMT_LOADABLE
#  include <nuttx/binfmt/binfmt.h>
#endif

#include "environ/environ.h"
#include "signal/signal.h"
#include "pthread/pthread.h"
#include "mqueue/mqueue.h"
#include "group/group.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_remove
 *
 * Description:
 *   Remove a group from the list of groups.
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

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
static void group_remove(FAR struct task_group_s *group)
{
  FAR struct task_group_s *curr;
  FAR struct task_group_s *prev;
  irqstate_t flags;

  /* Let's be especially careful while access the global task group list.
   * This is probably un-necessary.
   */

  flags = enter_critical_section();

  /* Find the task group structure */

  for (prev = NULL, curr = g_grouphead;
       curr && curr != group;
       prev = curr, curr = curr->flink);

  /* Did we find it?  If so, remove it from the list. */

  if (curr)
    {
      /* Do we remove it from mid-list?  Or from the head of the list? */

      if (prev)
        {
          prev->flink = curr->flink;
        }
      else
        {
          g_grouphead = curr->flink;
        }

      curr->flink = NULL;
    }

  leave_critical_section(flags);
}
#endif

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

static inline void group_release(FAR struct task_group_s *group)
{
#ifdef CONFIG_ARCH_ADDRENV
  save_addrenv_t oldenv;
  int i;
#endif

#if CONFIG_TLS_TASK_NELEM > 0
  task_tls_destruct();
#endif

  nxsem_destroy(&group->tg_info->ta_sem);
  group_free(group, group->tg_info);

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

  files_releaselist(&group->tg_filelist);

#ifdef CONFIG_FILE_STREAM
  /* Free resource held by the stream list */

  lib_stream_release(group);
#endif /* CONFIG_FILE_STREAM */

#ifndef CONFIG_DISABLE_ENVIRON
  /* Release all shared environment variables */

  env_release(group);
#endif

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_SHM)
  /* Release any resource held by shared memory virtual page allocator */

  shm_group_release(group);
#endif

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  /* Remove the group from the list of groups */

  group_remove(group);
#endif

#ifdef HAVE_GROUP_MEMBERS
  /* Release the members array */

  if (group->tg_members)
    {
      kmm_free(group->tg_members);
      group->tg_members = NULL;
    }
#endif

#ifdef CONFIG_MM_VM_MAP
  /* Destroy the vm_map list */

  vm_map_destroy(&group->tg_vm_map);
#endif

#if defined(CONFIG_FILE_STREAM) && defined(CONFIG_MM_KERNEL_HEAP)
  /* In a flat, single-heap build.  The stream list is part of the
   * group structure and, hence will be freed when the group structure
   * is freed.  Otherwise, it is separately allocated an must be
   * freed here.
   */

#  ifdef CONFIG_BUILD_KERNEL
  /* In the kernel build, the unprivileged process's stream list will be
   * allocated from with its per-process, private user heap. But in that
   * case, there is no reason to do anything here:  That allocation resides
   * in the user heap which which be completely freed when we destroy the
   * process's address environment.
   */

  if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
#  endif
    {
      /* But kernel threads are different in this build configuration: Their
       * stream lists were allocated from the common, global kernel heap and
       * must explicitly freed here.
       */

      group_free(group, group->tg_streamlist);
    }
#endif

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

#ifdef CONFIG_ARCH_ADDRENV
  /* Switch the addrenv and also save the current addrenv */

  up_addrenv_select(&group->tg_addrenv, &oldenv);

  /* Destroy the group address environment */

  up_addrenv_destroy(&group->tg_addrenv);

  /* Mark no address environment */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (group == g_group_current[i])
        {
          g_group_current[i] = NULL;
        }
    }

  /* Restore the previous addrenv */

  up_addrenv_restore(&oldenv);
#endif

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* If there are threads waiting for this group to be freed, then we cannot
   * yet free the memory resources.  Instead just mark the group deleted
   * and wait for those threads complete their waits.
   */

  if (group->tg_nwaiters > 0)
    {
      group->tg_flags |= GROUP_FLAG_DELETED;
    }
  else
#endif
    {
      /* Release the group container itself */

      kmm_free(group);
    }
}

/****************************************************************************
 * Name: group_removemember
 *
 * Description:
 *   Remove a member from a group.
 *
 * Input Parameters:
 *   group - The group from which to remove the member.
 *   pid - The member to be removed.
 *
 * Returned Value:
 *   On success, returns the number of members remaining in the group (>=0).
 *   Can fail only if the member is not found in the group.  On failure,
 *   returns -ENOENT
 *
 * Assumptions:
 *   Called during task deletion and also from the reparenting logic, both
 *   in a safe context.  No special precautions are required here.
 *
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
static inline void group_removemember(FAR struct task_group_s *group,
                                      pid_t pid)
{
  irqstate_t flags;
  int i;

  DEBUGASSERT(group);

  /* Find the member in the array of members and remove it */

  for (i = 0; i < group->tg_nmembers; i++)
    {
      /* Does this member have the matching pid */

      if (group->tg_members[i] == pid)
        {
          /* Remove the member from the array of members.  This must be an
           * atomic operation because the member array may be accessed from
           * interrupt handlers (read-only).
           */

          flags = enter_critical_section();
          group->tg_members[i] = group->tg_members[group->tg_nmembers - 1];
          group->tg_nmembers--;
          leave_critical_section(flags);
        }
    }
}
#endif /* HAVE_GROUP_MEMBERS */

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

#ifdef HAVE_GROUP_MEMBERS
void group_leave(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb);

  /* Make sure that we have a group. */

  group = tcb->group;
  if (group)
    {
      /* Remove the member from group.  This function may be called
       * during certain error handling before the PID has been
       * added to the group.  In this case tcb->pid will be uninitialized
       * group_removemember() will fail.
       */

      group_removemember(group, tcb->pid);

      /* Have all of the members left the group? */

      if (group->tg_nmembers == 0)
        {
          /* Yes.. Release all of the resource held by the task group */

          group_release(group);
        }

      /* In any event, we can detach the group from the TCB so that we won't
       * do this again.
       */

      tcb->group = NULL;
    }
}

#else /* HAVE_GROUP_MEMBERS */

void group_leave(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb);

  /* Make sure that we have a group */

  group = tcb->group;
  if (group)
    {
      /* Yes, we have a group.. Is this the last member of the group? */

      if (group->tg_nmembers > 1)
        {
          /* No.. just decrement the number of members in the group */

          group->tg_nmembers--;
        }

      /* Yes.. that was the last member remaining in the group */

      else
        {
          /* Release all of the resource held by the task group */

          group_release(group);
        }

      /* In any event, we can detach the group from the TCB so we won't do
       * this again.
       */

      tcb->group = NULL;
    }
}

#endif /* HAVE_GROUP_MEMBERS */
