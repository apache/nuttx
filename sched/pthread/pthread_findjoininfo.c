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
#include <debug.h>

#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_createjoininfo
 *
 * Description:
 *    Allocate a detachable structure to support pthread_join logic and add
 *    the joininfo to the thread.
 *
 * Input Parameters:
 *   ptcb
 *
 * Returned Value:
 *   joininfo point.
 *
 * Assumptions:
 *
 ****************************************************************************/

static FAR struct join_s *
pthread_createjoininfo(FAR struct pthread_tcb_s *ptcb)
{
  FAR struct join_s *pjoin;

  /* Allocate a detachable structure to support pthread_join logic */

  pjoin = (FAR struct join_s *)kmm_zalloc(sizeof(struct join_s));
  if (!pjoin)
    {
      serr("ERROR: Failed to allocate join\n");
      return NULL;
    }

  pjoin->thread = (pthread_t)ptcb->cmn.pid;

  /* Initialize the semaphore in the join structure to zero. */

  if (nxsem_init(&pjoin->exit_sem, 0, 0) < 0)
    {
      kmm_free(pjoin);
      return NULL;
    }
  else
    {
      FAR struct task_group_s *group = ptcb->cmn.group;

      /* Attach the join info to the TCB. */

      ptcb->joininfo = (FAR void *)pjoin;

      pjoin->next = NULL;
      if (!group->tg_jointail)
        {
          group->tg_joinhead = pjoin;
        }
      else
        {
          group->tg_jointail->next = pjoin;
        }

      group->tg_jointail = pjoin;
    }

  return pjoin;
}

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

  if (pjoin == NULL)
    {
      FAR struct tcb_s *tcb = nxsched_get_tcb((pthread_t)pid);

      if (tcb != NULL && (tcb->flags & TCB_FLAG_DETACHED) == 0 &&
          (tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
        {
          pjoin = pthread_createjoininfo((FAR struct pthread_tcb_s *)tcb);
        }
    }

  return pjoin;
}
