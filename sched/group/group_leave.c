/****************************************************************************
 *  sched/group/group_leave.c
 *
 *   Copyright (C) 2013-2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/lib/lib.h>

#ifdef CONFIG_BINFMT_LOADABLE
#  include <nuttx/binfmt/binfmt.h>
#endif

#include "environ/environ.h"
#include "signal/signal.h"
#include "pthread/pthread.h"
#include "mqueue/mqueue.h"
#include "group/group.h"

#ifdef HAVE_TASK_GROUP

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
  /* Free all un-reaped child exit status */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  group_removechildren(group);
#endif

#ifndef CONFIG_DISABLE_SIGNALS
  /* Release pending signals */

  nxsig_release(group);
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Release pthread resources */

  pthread_release(group);
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Free all file-related resources now.  We really need to close files as
   * soon as possible while we still have a functioning task.
   */

  /* Free resources held by the file descriptor list */

  files_releaselist(&group->tg_filelist);

#if CONFIG_NFILE_STREAMS > 0
  /* Free resource held by the stream list */

  lib_stream_release(group);

#endif /* CONFIG_NFILE_STREAMS */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  /* Free resource held by the socket list */

  net_releaselist(&group->tg_socketlist);
#endif /* CONFIG_NSOCKET_DESCRIPTORS */

#ifndef CONFIG_DISABLE_ENVIRON
  /* Release all shared environment variables */

  env_release(group);
#endif

#ifndef CONFIG_DISABLE_MQUEUE
  /* Close message queues opened by members of the group */

  nxmq_release(group);
#endif

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_SHM)
  /* Release any resource held by shared memory virtual page allocator */

  (void)shm_group_release(group);
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Destroy the group address environment */

  (void)up_addrenv_destroy(&group->tg_addrenv);

  /* Mark no address environment */

  g_gid_current = 0;
#endif

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
  /* Remove the group from the list of groups */

  group_remove(group);
#endif

#ifdef HAVE_GROUP_MEMBERS
  /* Release the members array */

  if (group->tg_members)
    {
      sched_kfree(group->tg_members);
      group->tg_members = NULL;
    }
#endif

#if CONFIG_NFILE_STREAMS > 0 && defined(CONFIG_MM_KERNEL_HEAP)
  /* In a flat, single-heap build.  The stream list is part of the
   * group structure and, hence will be freed when the group structure
   * is freed.  Otherwise, it is separately allocated an must be
   * freed here.
   */

#  if defined(CONFIG_BUILD_PROTECTED)
  /* In the protected build, the task's stream list is always allocated
   * and freed from the single, global user allocator.
   */

  sched_ufree(group->tg_streamlist);

#  elif defined(CONFIG_BUILD_KERNEL)
  /* In the kernel build, the unprivileged process' stream list will be
   * allocated from with its per-process, private user heap. But in that
   * case, there is no reason to do anything here:  That allocation resides
   * in the user heap which which be completely freed when we destroy the
   * process' address environment.
   */

  if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
    {
      /* But kernel threads are different in this build configuration: Their
       * stream lists were allocated from the common, global kernel heap and
       * must explicitly freed here.
       */

      sched_kfree(group->tg_streamlist);
    }

#  endif
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

      sched_kfree(group);
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
static inline void group_removemember(FAR struct task_group_s *group, pid_t pid)
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
#endif /* HAVE_TASK_GROUP */
