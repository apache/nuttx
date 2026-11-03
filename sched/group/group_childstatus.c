/****************************************************************************
 * sched/group/group_childstatus.c
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
#include <nuttx/debug.h>

#include "sched/sched.h"
#include "group/group.h"

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_PREALLOC_CHILDSTATUS) || CONFIG_PREALLOC_CHILDSTATUS == 0
#  undef  CONFIG_PREALLOC_CHILDSTATUS
#  define CONFIG_PREALLOC_CHILDSTATUS 16
#endif

#ifndef CONFIG_DEBUG_INFO
#  undef CONFIG_DEBUG_CHILDSTATUS
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Globals are maintained in a structure to minimize name collisions. */

struct child_pool_s
{
  struct child_status_s alloc[CONFIG_PREALLOC_CHILDSTATUS];
  FAR struct child_status_s *freelist;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct child_pool_s g_child_pool;
static spinlock_t g_child_pool_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_dump_children
 *
 * Description:
 *   Dump all of the children when the part TCB list is modified.
 *
 * Input Parameters:
 *   group - The task group containing the child status.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called early in initialization.  No special precautions are required.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_CHILDSTATUS
static void group_dump_children(FAR struct task_group_s *group,
                                FAR const char *msg)
{
  FAR struct child_status_s *child;
  int i;

  _info("Task group=%p: %s\n", group, msg);
  for (i = 0, child = group->tg_children; child; i++, child = child->flink)
    {
      _info("  %d. ch_flags=%02x ch_pid=%d ch_status=%d\n",
            i, child->ch_flags, child->ch_pid, child->ch_status);
    }
}
#else
#  define group_dump_children(t,m)
#endif

/****************************************************************************
 * Name: group_child_pgid
 *
 * Description:
 *   Return the process group ID of a child.  A child that exited is in the
 *   process group that it had when its main thread exited.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_WAITPID
static pid_t group_child_pgid(FAR struct child_status_s *child)
{
#ifdef CONFIG_SCHED_PROCESS_GROUPS
  FAR struct task_group_s *chgrp;

  if ((child->ch_flags & CHILD_FLAG_EXITED) == 0)
    {
      chgrp = task_getgroup(child->ch_pid);
      if (chgrp != NULL)
        {
          return chgrp->tg_pgid;
        }
    }

  return child->ch_pgid;
#else
  return child->ch_pid;
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_initialize
 *
 * Description:
 *   Initialize task related status.  At present, this includes only the
 *   initialize of the child status pool.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called early in initialization.  No special precautions are required.
 *
 ****************************************************************************/

void task_initialize(void)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *prev;
  int i;

  /* Save all of the child status structures in a free list */

  prev = &g_child_pool.alloc[0];
  g_child_pool.freelist = prev;
  for (i = 0; i < CONFIG_PREALLOC_CHILDSTATUS; i++)
    {
      curr        = &g_child_pool.alloc[i];
      prev->flink = curr;
      prev        = curr;
    }
}

/****************************************************************************
 * Name: group_alloc_child
 *
 * Description:
 *   Allocate a child status structure by removing the next entry from a
 *   free list.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned when memory allocation fails.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

FAR struct child_status_s *group_alloc_child(void)
{
  FAR struct child_status_s *ret;
  irqstate_t flags;

  /* Return the status block at the head of the free list */

  flags = spin_lock_irqsave(&g_child_pool_lock);
  ret = g_child_pool.freelist;
  if (ret)
    {
      g_child_pool.freelist = ret->flink;
      ret->flink            = NULL;
      spin_unlock_irqrestore(&g_child_pool_lock, flags);
    }
  else
    {
      spin_unlock_irqrestore(&g_child_pool_lock, flags);
      ret = kmm_zalloc(sizeof(*ret));
    }

  return ret;
}

/****************************************************************************
 * Name: group_free_child
 *
 * Description:
 *   Release a child status structure by returning it to a free list.
 *
 * Input Parameters:
 *   status - The child status structure to be freed.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

void group_free_child(FAR struct child_status_s *child)
{
  irqstate_t flags;

  /* Return the child status structure to the free list  */

  if (child)
    {
      flags = spin_lock_irqsave(&g_child_pool_lock);
      child->flink          = g_child_pool.freelist;
      g_child_pool.freelist = child;
      spin_unlock_irqrestore(&g_child_pool_lock, flags);
    }
}

/****************************************************************************
 * Name: group_add_child
 *
 * Description:
 *   Add a child status structure in the given TCB.
 *
 * Input Parameters:
 *   group  - The task group for the child status.
 *   child  - The structure to be added
 *
 * Returned Value:
 *   N
 *
 * Assumptions:
 *   Called during task creation processing in a safe context.  No special
 *   precautions are required here.
 *
 ****************************************************************************/

void group_add_child(FAR struct task_group_s *group,
                     FAR struct child_status_s *child)
{
  /* Add the entry into the TCB list of children */

  child->flink  = group->tg_children;
  group->tg_children = child;

  group_dump_children(group, "group_add_child");
}

/****************************************************************************
 * Name: group_find_child
 *
 * Description:
 *   Find a child status structure in the given task group.  A reference to
 *   the child structure is returned, but the child remains in the group's
 *   list of children.
 *
 * Input Parameters:
 *   group - The ID of the parent task group to containing the child status.
 *   pid - The ID of the child to find.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if there is child status structure for that pid in the TCB.
 *
 * Assumptions:
 *   Called during SIGCHLD processing in a safe context.  No special
 *   precautions are required here.
 *
 ****************************************************************************/

FAR struct child_status_s *group_find_child(FAR struct task_group_s *group,
                                            pid_t pid)
{
  FAR struct child_status_s *child;
  irqstate_t flags;

  DEBUGASSERT(group);

  /* Find the status structure with the matching PID  */

  flags = spin_lock_irqsave(&group->tg_lock);
  for (child = group->tg_children; child; child = child->flink)
    {
      if (child->ch_pid == pid)
        {
          spin_unlock_irqrestore(&group->tg_lock, flags);
          return child;
        }
    }

  spin_unlock_irqrestore(&group->tg_lock, flags);
  return NULL;
}

/****************************************************************************
 * Name: group_child_event
 *
 * Description:
 *   Return the event of a child that waitpid() or waitid() can report with
 *   these options: the child exited (WEXITED), stopped (WUNTRACED or
 *   WSTOPPED) or continued (WCONTINUED), and the event was not reported
 *   yet.  An exit comes before the other events.
 *
 * Input Parameters:
 *   child   - The child status structure.
 *   options - The options of waitpid() or waitid().
 *
 * Returned Value:
 *   CHILD_FLAG_EXITED, CHILD_FLAG_STOPPED or CHILD_FLAG_CONTINUED, or 0 if
 *   the child has no event to report.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_WAITPID
uint8_t group_child_event(FAR struct child_status_s *child, int options)
{
  if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
    {
      return (options & WEXITED) != 0 ? CHILD_FLAG_EXITED : 0;
    }

  if ((child->ch_flags & CHILD_FLAG_STOPPED) != 0 &&
      (options & (WUNTRACED | WSTOPPED)) != 0)
    {
      return CHILD_FLAG_STOPPED;
    }

  if ((child->ch_flags & CHILD_FLAG_CONTINUED) != 0 &&
      (options & WCONTINUED) != 0)
    {
      return CHILD_FLAG_CONTINUED;
    }

  return 0;
}

/****************************************************************************
 * Name: group_wait_child
 *
 * Description:
 *   Find a child that idtype and id select and that has an event to report
 *   (see group_child_event()).  P_ALL selects all children, P_PID the child
 *   with the process ID id, and P_PGID the children in the process group
 *   id.  A child that exited is in the process group that it had when its
 *   main thread exited.  Without CONFIG_SCHED_PROCESS_GROUPS, a process
 *   group contains only one process.
 *
 * Input Parameters:
 *   group   - The parent task group.
 *   idtype  - P_ALL, P_PID or P_PGID.
 *   id      - The process ID or the process group ID.
 *   options - The options of waitpid() or waitid().
 *   found   - Return true if idtype and id select a child.
 *
 * Returned Value:
 *   The child status structure, or NULL if no child that idtype and id
 *   select has an event to report.
 *
 * Assumptions:
 *   Called from a critical section.
 *
 ****************************************************************************/

FAR struct child_status_s *group_wait_child(FAR struct task_group_s *group,
                                            idtype_t idtype, pid_t id,
                                            int options, FAR bool *found)
{
  FAR struct child_status_s *child;
  irqstate_t flags;

  *found = false;

  flags = spin_lock_irqsave(&group->tg_lock);
  for (child = group->tg_children; child; child = child->flink)
    {
      if (idtype == P_PID && child->ch_pid != id)
        {
          continue;
        }

      if (idtype == P_PGID && group_child_pgid(child) != id)
        {
          continue;
        }

      *found = true;
      if (group_child_event(child, options) != 0)
        {
          break;
        }
    }

  spin_unlock_irqrestore(&group->tg_lock, flags);
  return child;
}
#endif /* CONFIG_SCHED_WAITPID */

/****************************************************************************
 * Name: group_remove_child
 *
 * Description:
 *   Remove one child structure from a task group.  The child is removed, but
 *   is not yet freed.  group_free_child must be called in order to free the
 *   child status structure.
 *
 * Input Parameters:
 *   group - The task group containing the child status.
 *   pid - The ID of the child to find.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a child status structure.  NULL is
 *   returned if there is child status structure for that pid in the TCB.
 *
 * Assumptions:
 *   Called during SIGCHLD processing in a safe context.  No special
 *   precautionsare required here.
 *
 ****************************************************************************/

FAR struct child_status_s *group_remove_child(FAR struct task_group_s *group,
                                              pid_t pid)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *prev;
  irqstate_t flags;

  DEBUGASSERT(group);

  /* Find the status structure with the matching PID */

  flags = spin_lock_irqsave(&group->tg_lock);
  for (prev = NULL, curr = group->tg_children;
       curr;
       prev = curr, curr = curr->flink)
    {
      if (curr->ch_pid == pid)
        {
          break;
        }
    }

  /* Did we find it?  If so, remove it from the group. */

  if (curr)
    {
      /* Do we remove it from mid-list?  Or from the head of the list? */

      if (prev)
        {
          prev->flink = curr->flink;
        }
      else
        {
          group->tg_children = curr->flink;
        }

      curr->flink = NULL;
      spin_unlock_irqrestore(&group->tg_lock, flags);
      group_dump_children(group, "group_remove_child");
      return curr;
    }

  spin_unlock_irqrestore(&group->tg_lock, flags);
  return curr;
}

/****************************************************************************
 * Name: group_remove_children
 *
 * Description:
 *   Remove and free all child structure from the task group.
 *
 * Input Parameters:
 *   group - The task group containing the child status.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Called during task exit processing in a safe context.  No special
 *   precautions are required here.
 *
 ****************************************************************************/

void group_remove_children(FAR struct task_group_s *group)
{
  FAR struct child_status_s *curr;
  FAR struct child_status_s *next;
  irqstate_t flags;

  /* Remove all child structures for the TCB and return them to the
   * freelist.
   */

  flags = spin_lock_irqsave(&group->tg_lock);
  for (curr = group->tg_children; curr; curr = next)
    {
      next = curr->flink;
      group_free_child(curr);
    }

  group->tg_children = NULL;
  spin_unlock_irqrestore(&group->tg_lock, flags);

  group_dump_children(group, "group_remove_children");
}

#endif /* CONFIG_SCHED_HAVE_PARENT && CONFIG_SCHED_CHILD_STATUS */
