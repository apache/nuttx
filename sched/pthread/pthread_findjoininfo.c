/****************************************************************************
 * sched/pthread/pthread_findjoininfo.c
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

#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: thread_findjoininfo
 *
 * Description:
 *   Find a join structure in a local data set.
 *
 * Input Parameters:
 *   group - The group that the pid is (or was) a member of
 *   pid - The ID of the pthread
 *
 * Returned Value:
 *   None or pointer to the found entry.
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ****************************************************************************/

FAR struct join_s *pthread_findjoininfo(FAR struct task_group_s *group,
                                        pid_t pid)
{
  FAR struct join_s *pjoin;

  DEBUGASSERT(group);

  /* Find the entry with the matching pid */

  for (pjoin = group->tg_joinhead;
       (pjoin && (pid_t)pjoin->thread != pid);
       pjoin = pjoin->next);

  /* and return it */

  return pjoin;
}
