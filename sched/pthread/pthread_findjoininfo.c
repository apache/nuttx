/****************************************************************************
 * sched/pthread/pthread_findjoininfo.c
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/nuttx.h>

#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_findjoininfo
 *
 * Description:
 *   Find a join structure in a local data set.
 *
 * Input Parameters:
 *   group - The group that the pid is (or was) a member of
 *   pid - The ID of the pthread
 *
 * Output Parameters:
 *   pjoin - None or pointer to the found entry
 *
 * Returned Value:
 *   0 if successful.  Otherwise, one of the following error codes:
 *
 *   EINVAL  The value specified by thread does not refer to joinable
 *           thread.
 *   ESRCH   No thread could be found corresponding to that specified by the
 *           given thread ID.
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ****************************************************************************/

int pthread_findjoininfo(FAR struct task_group_s *group, pid_t pid,
                         FAR struct task_join_s **pjoin, bool create)
{
  FAR struct task_join_s *join;
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;

  nxrmutex_lock(&group->tg_mutex);

  sq_for_every_safe(&group->tg_joinqueue, curr, next)
    {
      join = container_of(curr, struct task_join_s, entry);

      if (join->pid == pid)
        {
          goto found;
        }
    }

  nxrmutex_unlock(&group->tg_mutex);

  if (!create)
    {
      return EINVAL;
    }

  join = kmm_zalloc(sizeof(struct task_join_s));
  if (join == NULL)
    {
      return ENOMEM;
    }

  join->pid = pid;

  nxrmutex_lock(&group->tg_mutex);

  sq_addfirst(&join->entry, &group->tg_joinqueue);

found:
  nxrmutex_unlock(&group->tg_mutex);

  *pjoin = join;

  return OK;
}
