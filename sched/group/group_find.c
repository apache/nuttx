/****************************************************************************
 *  sched/group/group_find.c
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
#include <nuttx/sched.h>

#include "group/group.h"
#include "environ/environ.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_findby_grpid
 *
 * Description:
 *   Given a group ID, find the group task structure with that ID.  IDs are
 *   used instead of pointers to group structures.  This is done because a
 *   group can disappear at any time leaving a stale pointer; an ID is
 *   cleaner because if the group disappears, this function will fail
 *   gracefully.
 *
 * Input Parameters:
 *   grpid - The group ID to find.
 *
 * Returned Value:
 *   On success, a pointer to the group task structure is returned.  This
 *   function can fail only if there is no group that corresponds to the
 *   group ID.
 *
 * Assumptions:
 *   Called during when signally tasks in a safe context.  No special
 *   precautions should be required here.  However, extra care is taken when
 *   accessing the global g_grouphead list.
 *
 ****************************************************************************/

#if defined(HAVE_GROUP_MEMBERS) || defined(CONFIG_ARCH_ADDRENV)
FAR struct task_group_s *group_findby_grpid(grpid_t grpid)
{
  FAR struct task_group_s *group;
  irqstate_t flags;

  /* Find the status structure with the matching GID  */

  flags = enter_critical_section();
  for (group = g_grouphead; group; group = group->flink)
    {
      if (group->tg_grpid == grpid)
        {
          leave_critical_section(flags);
          return group;
        }
    }

  leave_critical_section(flags);
  return NULL;
}
#endif

/****************************************************************************
 * Name: group_findbypid
 *
 * Description:
 *   Given a task ID, find the group task structure with was started by that
 *   task ID.  That task's ID is retained in the group as tg_task and will
 *   be remember even if the main task thread leaves the group.
 *
 * Input Parameters:
 *   pid - The task ID of the main task thread.
 *
 * Returned Value:
 *   On success, a pointer to the group task structure is returned.  This
 *   function can fail only if there is no group that corresponds to the
 *   task ID.
 *
 * Assumptions:
 *   Called during when signally tasks in a safe context.  No special
 *   precautions should be required here.  However, extra care is taken when
 *   accessing the global g_grouphead list.
 *
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
FAR struct task_group_s *group_findbypid(pid_t pid)
{
  FAR struct task_group_s *group;
  irqstate_t flags;

  /* Find the status structure with the matching PID  */

  flags = enter_critical_section();
  for (group = g_grouphead; group; group = group->flink)
    {
      if (group->tg_task == pid)
        {
          leave_critical_section(flags);
          return group;
        }
    }

  leave_critical_section(flags);
  return NULL;
}
#endif
