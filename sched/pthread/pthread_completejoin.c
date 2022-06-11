/****************************************************************************
 * sched/pthread/pthread_completejoin.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_notifywaiters
 *
 * Description:
 *   Notify all other threads waiting in phread join for this thread's
 *   exit data.  This must  be done by the child at child thread
 *   destruction time.
 *
 ****************************************************************************/

static bool pthread_notifywaiters(FAR struct join_s *pjoin)
{
  int ntasks_waiting;
  int status;

  sinfo("pjoin=0x%p\n", pjoin);

  /* Are any tasks waiting for our exit value? */

  status = nxsem_get_value(&pjoin->exit_sem, &ntasks_waiting);
  if (status == OK && ntasks_waiting < 0)
    {
      /* Set the data semaphore so that this thread will be
       * awakened when all waiting tasks receive the data
       */

      nxsem_init(&pjoin->data_sem, 0, (ntasks_waiting + 1));

      /* Post the semaphore to restart each thread that is waiting
       * on the semaphore
       */

      do
        {
          status = pthread_sem_give(&pjoin->exit_sem);
          if (status == OK)
            {
              status = nxsem_get_value(&pjoin->exit_sem, &ntasks_waiting);
            }
        }
      while (ntasks_waiting < 0 && status == OK);

      /* Now wait for all these restarted tasks to obtain the return
       * value.
       */

      nxsem_wait_uninterruptible(&pjoin->data_sem);
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: pthread_removejoininfo
 *
 * Description:
 *   Remove a join structure from the local data set.
 *
 * Input Parameters:
 *   pid
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ****************************************************************************/

static void pthread_removejoininfo(FAR struct task_group_s *group,
                                   pid_t pid)
{
  FAR struct join_s *prev;
  FAR struct join_s *join;

  /* Find the entry with the matching pid */

  for (prev = NULL, join = group->tg_joinhead;
       (join && (pid_t)join->thread != pid);
       prev = join, join = join->next);

  /* Remove it from the data set. */

  /* First check if this is the entry at the head of the list. */

  if (join)
    {
      if (!prev)
        {
          /* Check if this is the only entry in the list */

          if (!join->next)
            {
              group->tg_joinhead = NULL;
              group->tg_jointail = NULL;
            }

          /* Otherwise, remove it from the head of the list */

          else
            {
              group->tg_joinhead = join->next;
            }
        }

      /* It is not at the head of the list, check if it is at the tail. */

      else if (!join->next)
        {
          group->tg_jointail = prev;
          prev->next = NULL;
        }

      /* No, remove it from the middle of the list. */

      else
        {
          prev->next = join->next;
        }
    }
}

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
  FAR struct task_group_s *group = task_getgroup(pid);
  FAR struct join_s *pjoin;

  sinfo("pid=%d exit_value=%p group=%p\n", pid, exit_value, group);
  DEBUGASSERT(group);

  /* First, find thread's structure in the private data set. */

  nxsem_wait_uninterruptible(&group->tg_joinsem);
  pjoin = pthread_findjoininfo(group, pid);
  if (!pjoin)
    {
      serr("ERROR: Could not find join info, pid=%d\n", pid);
      pthread_sem_give(&group->tg_joinsem);
      return ERROR;
    }
  else
    {
      bool waiters;

      /* Save the return exit value in the thread structure. */

      pjoin->terminated = true;
      pjoin->exit_value = exit_value;

      /* Notify waiters of the availability of the exit value */

      waiters = pthread_notifywaiters(pjoin);

      /* If there are no waiters and if the thread is marked as detached.
       * then discard the join information now.  Otherwise, the pthread
       * join logic will call pthread_destroyjoin() when all of the threads
       * have sampled the exit value.
       */

      if (!waiters && pjoin->detached)
        {
          pthread_destroyjoin(group, pjoin);
        }

      /* Giving the following semaphore will allow the waiters
       * to call pthread_destroyjoin.
       */

      pthread_sem_give(&group->tg_joinsem);
    }

  return OK;
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
 *   The caller holds tg_joinsem
 *
 ****************************************************************************/

void pthread_destroyjoin(FAR struct task_group_s *group,
                         FAR struct join_s *pjoin)
{
  sinfo("pjoin=0x%p\n", pjoin);

  /* Remove the join info from the set of joins */

  pthread_removejoininfo(group, (pid_t)pjoin->thread);

  /* Destroy its semaphores */

  nxsem_destroy(&pjoin->data_sem);
  nxsem_destroy(&pjoin->exit_sem);

  /* And deallocate the pjoin structure */

  kmm_free(pjoin);
}
