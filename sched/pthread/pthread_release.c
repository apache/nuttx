/****************************************************************************
 * sched/pthread/pthread_release.c
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
#include <debug.h>

#include <nuttx/nuttx.h>
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
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;

  /* Visit and delete each join structure still in the list.  Since we
   * are last exiting thread of the group, no special protection should
   * be required.
   */

  sq_for_every_safe(&group->tg_joinqueue, curr, next)
    {
      /* Deallocate the join structure */

      kmm_free(container_of(curr, struct task_join_s, entry));
    }
}
