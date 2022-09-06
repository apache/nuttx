/****************************************************************************
 * sched/pthread/pthread_release.c
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
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_release
 *
 * Description:
 *   Release pthread resources from the task group with the group
 *   terminated.
 *
 * Input Parameters:
 *   group = The task group containing the pthread resources to be
 *           released.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *
 ****************************************************************************/

void pthread_release(FAR struct task_group_s *group)
{
  FAR struct join_s *join;
  DEBUGASSERT(group);

  /* Visit and delete each join structure still in the list.  Since we
   * are last exiting thread of the group, no special protection should
   * be required.
   */

  while (group->tg_joinhead)
    {
      /* Remove the join from the head of the list. */

      join = group->tg_joinhead;
      group->tg_joinhead = join->next;

      /* Destroy the join semaphores */

      nxsem_destroy(&join->data_sem);
      nxsem_destroy(&join->exit_sem);

      /* And deallocate the join structure */

      kmm_free(join);
    }

  /* Destroy the join list mutex */

  nxmutex_destroy(&group->tg_joinlock);
}
