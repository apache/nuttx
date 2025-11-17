/****************************************************************************
 * sched/pthread/pthread_join.c
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
#include <unistd.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "pthread/pthread.h"

#ifdef CONFIG_DEBUG_FEATURES
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: is_thread_in_join_queue
 *
 * Description:
 *   Check if a specific thread is in another thread's wait queue.
 *
 * Input Parameters:
 *   joiner - Thread to look for in the wait queue.
 *   joinee - Thread whose wait queue to check.
 *
 * Returned Value:
 *   true if joiner is in joinee's wait queue, false otherwise.
 *
 ****************************************************************************/

static bool is_thread_in_join_queue(FAR struct tcb_s *joiner,
                                    FAR struct tcb_s *joinee)
{
  FAR struct tcb_s *tcb;

  sq_for_every_entry(&joinee->join_queue, tcb, struct tcb_s, join_entry)
    {
      if (tcb == joiner)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: is_thread_waiting_for_join
 *
 * Description:
 *   Check if a thread is currently waiting in a pthread_join operation.
 *
 * Input Parameters:
 *   joiner - Thread to check.
 *
 * Returned Value:
 *   true if thread is waiting in pthread_join, false otherwise.
 *
 ****************************************************************************/

static bool is_thread_waiting_for_join(FAR struct tcb_s *joiner)
{
  /* Check if the thread is waiting on its own join_sem */

  return joiner->waitobj == &joiner->join_sem;
}

/****************************************************************************
 * Name: find_thread_waiting_for_join
 *
 * Description:
 *   Find which thread a given thread is currently waiting for by examining
 *   all threads in the task group and their wait queues.
 *
 * Input Parameters:
 *   joiner - Thread to check.
 *
 * Returned Value:
 *   Pointer to the thread being waited for, or NULL if not waiting.
 *
 ****************************************************************************/

static FAR struct tcb_s *
find_thread_waiting_for_join(FAR struct tcb_s *joiner)
{
  FAR struct task_group_s *group = joiner->group;
  FAR struct tcb_s *joinee;

  /* Check if the thread is waiting for a join operation */

  if (!is_thread_waiting_for_join(joiner))
    {
      return NULL;
    }

  sq_for_every_entry(&group->tg_members, joinee, struct tcb_s, member)
    {
      /* Check if the thread is in this potential joinee's wait queue */

      if (is_thread_in_join_queue(joiner, joinee))
        {
          return joinee;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: is_thread_deadlock_for_join
 *
 * Description:
 *   Check if attempting to join the target thread would create a deadlock
 *   situation (circular waiting).
 *
 * Input Parameters:
 *   joiner - Thread attempting to execute pthread_join.
 *   joinee - Target thread to be joined.
 *
 * Returned Value:
 *   true if deadlock would occur, false otherwise.
 *
 ****************************************************************************/

static bool is_thread_deadlock_for_join(FAR struct tcb_s *joiner,
                                        FAR struct tcb_s *joinee)
{
  size_t depth = sq_count(&joinee->group->tg_members);

  /* Check for a circular wait */

  while (depth-- > 0)
    {
      joinee = find_thread_waiting_for_join(joinee);
      if (joinee == NULL)
        {
          return false;  /* Chain ended, no circular wait */
        }

      if (joinee == joiner)
        {
          return true;   /* Found joiner, circular wait detected */
        }
    }

  return false; /* No circular wait detected */
}
#endif /* CONFIG_DEBUG_FEATURES */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_join
 *
 * Description:
 *    A thread can await termination of another thread and retrieve the
 *    return value of the thread.
 *
 *    The caller's task/thread must belong to the same "task group" as the
 *    pthread is (or was) a member of.  The thread may or may not still
 *    be running.
 *
 * Input Parameters:
 *   thread
 *   pexit_value
 *
 * Returned Value:
 *   0 if successful.  Otherwise, one of the following error codes:
 *
 *   EINVAL  The value specified by thread does not refer to joinable
 *           thread.
 *   ESRCH   No thread could be found corresponding to that specified by the
 *           given thread ID.
 *   EDEADLK A deadlock was detected or the value of thread specifies the
 *           calling thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_join(pthread_t thread, FAR pthread_addr_t *pexit_value)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;
  FAR struct task_join_s *join;
  FAR struct tcb_s *tcb;
  int ret = OK;

  /* pthread_join() is a cancellation point */

  enter_cancellation_point();

  nxrmutex_lock(&group->tg_mutex);

  tcb = nxsched_get_tcb((pid_t)thread);
  if (tcb == NULL || (tcb->flags & TCB_FLAG_JOIN_COMPLETED) != 0)
    {
      ret = pthread_findjoininfo(group, (pid_t)thread, &join, false);
      if (ret == OK)
        {
          /* Destroy the join information after obtain the exit value */

          if (pexit_value != NULL)
            {
              *pexit_value = join->exit_value;
            }

          pthread_destroyjoin(group, join);
        }
      else
        {
          ret = ESRCH;
        }

      goto errout;
    }

  /* First make sure that this is not an attempt to join to
   * ourself.
   */

  if (tcb == rtcb)
    {
      ret = EDEADLK;
      goto errout;
    }

  /* Task was detached or not a pthread, return EINVAL */

  if ((tcb->group != group) ||
      (tcb->flags & TCB_FLAG_DETACHED) != 0)
    {
      ret = EINVAL;
      goto errout;
    }

  /* Relinquish the data set semaphore.  Since pre-emption is
   * disabled, we can be certain that no task has the
   * opportunity to run between the time we relinquish the
   * join semaphore and the time that we wait on the thread exit
   * semaphore.
   */

  sq_addfirst(&rtcb->join_entry, &tcb->join_queue);

#ifdef CONFIG_DEBUG_FEATURES
  if (is_thread_deadlock_for_join(rtcb, tcb))
    {
      sq_rem(&rtcb->join_entry, &tcb->join_queue);
      ret = EDEADLK;
      goto errout;
    }
#endif

  nxrmutex_unlock(&group->tg_mutex);

  /* Take the thread's thread exit semaphore.  We will sleep here
   * until the thread exits.  We need to exercise caution because
   * there could be multiple threads waiting here for the same
   * pthread to exit.
   */

  nxsem_wait_uninterruptible(&rtcb->join_sem);

  nxrmutex_lock(&group->tg_mutex);

  /* The thread has exited! Get the thread exit value */

  if (pexit_value != NULL)
    {
      *pexit_value = rtcb->join_val;
    }

errout:
  nxrmutex_unlock(&group->tg_mutex);

  leave_cancellation_point();

  if (pexit_value)
    {
      sinfo("Returning %d, exit_value %p\n", ret, *pexit_value);
    }
  else
    {
      sinfo("Returning %d\n", ret);
    }

  return ret;
}
