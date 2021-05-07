/****************************************************************************
 * sched/task/task_getgroup.c
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

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_getgroup
 *
 * Description:
 *   Given a task ID, return the group structure of this task.
 *
 * Input Parameters:
 *   pid - The task ID to use in the lookup.
 *
 * Returned Value:
 *   On success, a pointer to the group task structure is returned.  This
 *   function can fail only if there is no group that corresponds to the
 *   grouped ID.
 *
 * Assumptions:
 *   Called during when signally tasks in a safe context.  No special
 *   precautions should be required here.  However, extra care is taken when
 *   accessing the global g_grouphead list.
 *
 ****************************************************************************/

FAR struct task_group_s *task_getgroup(pid_t pid)
{
  FAR struct tcb_s *tcb = nxsched_get_tcb(pid);
  if (tcb)
    {
      return tcb->group;
    }

  return NULL;
}
