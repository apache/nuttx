/****************************************************************************
 * sched/pthread/pthread_completejoin.c
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

#include <nuttx/nuttx.h>
#include <sys/types.h>
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_completejoin
 *
 * Description:
 *   A thread has been terminated -- either by returning, calling
 *   pthread_exit(), or through pthread_cancel().  In any event, we must
 *   complete any pending join events.
 *
 * Input Parameters:
 *   exit_value
 *
 * Returned Value:
 *   OK unless there is no join information associated with the pid.
 *   This could happen, for example, if a task started with task_create()
 *   calls pthread_exit().
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_completejoin(pid_t pid, FAR void *exit_value)
{
  FAR struct tcb_s *tcb = nxsched_get_tcb(pid);
  FAR struct task_group_s *group = tcb->group;
  FAR struct task_join_s *join;
  FAR struct tcb_s *wtcb;
  FAR sq_entry_t *curr;
  FAR sq_entry_t *next;
  int ret = OK;

  sinfo("pid=%d exit_value=%p\n", pid, exit_value);

  nxrmutex_lock(&group->tg_mutex);

  if (!sq_empty(&tcb->join_queue))
    {
      sq_for_every_safe(&tcb->join_queue, curr, next)
        {
          /* Remove join entry from queue */

          sq_rem(curr, &tcb->join_queue);

          /* Get tcb entry which waiting for the join */

          wtcb = container_of(curr, struct tcb_s, join_entry);

          /* Save the return exit value in the thread structure. */

          wtcb->join_val = exit_value;

          /* Notify waiters of the availability of the exit value */

          nxsem_post(&wtcb->join_sem);
        }
    }
  else if (!sq_is_singular(&tcb->group->tg_members) &&
           (tcb->flags & TCB_FLAG_DETACHED) == 0)
    {
      ret = pthread_findjoininfo(tcb->group, pid, &join, true);
      if (ret == OK)
        {
          join->exit_value = exit_value;
        }
    }

  tcb->flags |= TCB_FLAG_JOIN_COMPLETED;

  nxrmutex_unlock(&group->tg_mutex);

  return ret;
}

/****************************************************************************
 * Name: pthread_destroyjoin
 *
 * Description:
 *   This is called from pthread_completejoin if the join info was
 *   detached or from pthread_join when the last waiting thread has
 *   received the thread exit info.
 *
 *   Or it may never be called if the join info was never detached or if
 *   no thread ever calls pthread_join.  In case, there is a memory leak!
 *
 * Assumptions:
 *   The caller holds tg_mutex
 *
 ****************************************************************************/

void pthread_destroyjoin(FAR struct task_group_s *group,
                         FAR struct task_join_s *pjoin)
{
  sinfo("pjoin=%p\n", pjoin);

  /* Remove the join info from the set of joins */

  nxrmutex_lock(&group->tg_mutex);
  sq_rem(&pjoin->entry, &group->tg_joinqueue);
  nxrmutex_unlock(&group->tg_mutex);

  /* And deallocate the pjoin structure */

  kmm_free(pjoin);
}
