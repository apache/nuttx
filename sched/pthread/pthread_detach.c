/****************************************************************************
 * sched/pthread/pthread_detach.c
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
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_detach
 *
 * Description:
 *    A thread object may be "detached" to specify that the return value
 *    and completion status will not be requested.
 *
 *    The caller's task/thread must belong to the same "task group" as the
 *    pthread is (or was) a member of.  The thread may or may not still
 *    be running.
 *
 * Input Parameters:
 *   thread
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_detach(pthread_t thread)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  FAR struct task_join_s *join;
  FAR struct tcb_s *tcb;
  int ret;

  nxrmutex_lock(&group->tg_mutex);

  tcb = nxsched_get_tcb((pid_t)thread);
  if (tcb == NULL || (tcb->flags & TCB_FLAG_JOIN_COMPLETED) != 0)
    {
      /* Destroy the join information */

      ret = pthread_findjoininfo(group, (pid_t)thread, &join, false);
      if (ret == OK)
        {
          pthread_destroyjoin(group, join);
        }
      else
        {
          ret = ESRCH;
        }

      goto errout;
    }

  if ((group != tcb->group) ||
      (tcb->flags & TCB_FLAG_DETACHED) != 0)
    {
      ret = EINVAL;
    }
  else
    {
      tcb->flags |= TCB_FLAG_DETACHED;
      ret = OK;
    }

errout:
  nxrmutex_unlock(&group->tg_mutex);

  sinfo("Returning %d\n", ret);
  return ret;
}
