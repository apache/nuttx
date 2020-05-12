/****************************************************************************
 * sched/pthread/pthread_join.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

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
  FAR struct join_s *pjoin;
  int ret;

  sinfo("thread=%d group=%p\n", thread, group);
  DEBUGASSERT(group);

  /* pthread_join() is a cancellation point */

  enter_cancellation_point();

  /* First make sure that this is not an attempt to join to
   * ourself.
   */

  if ((pid_t)thread == getpid())
    {
      leave_cancellation_point();
      return EDEADLK;
    }

  /* Make sure no other task is mucking with the data structures
   * while we are performing the following operations.  NOTE:
   * we can be also sure that pthread_exit() will not execute
   * because it will also attempt to get this semaphore.
   */

  pthread_sem_take(&group->tg_joinsem, NULL, false);

  /* Find the join information associated with this thread.
   * This can fail for one of three reasons:  (1) There is no
   * thread associated with 'thread,' (2) the thread is a task
   * and does not have join information, or (3) the thread
   * was detached and has exited.
   */

  pjoin = pthread_findjoininfo(group, (pid_t)thread);
  if (pjoin == NULL)
    {
      /* Determine what kind of error to return */

      FAR struct tcb_s *tcb = nxsched_get_tcb((pthread_t)thread);

      swarn("WARNING: Could not find thread data\n");

      /* Case (1) or (3) -- we can't tell which.  Assume (3) */

      if (tcb == NULL)
        {
          ret = ESRCH;
        }

      /* The thread is still active but has no join info.  In that
       * case, it must be a task and not a pthread.
       */

      else
        {
          ret = EINVAL;
        }

      pthread_sem_give(&group->tg_joinsem);
    }
  else
    {
      /* We found the join info structure.  Increment for the reference
       * to the join structure that we have.  This will keep things
       * stable for we have to do
       */

      sched_lock();
      pjoin->crefs++;

      /* Check if the thread is still running.  If not, then things are
       * simpler.  There are still race conditions to be concerned with.
       * For example, there could be multiple threads executing in the
       * 'else' block below when we enter!
       */

      if (pjoin->terminated)
        {
          sinfo("Thread has terminated\n");

          /* Get the thread exit value from the terminated thread. */

          if (pexit_value)
            {
              sinfo("exit_value=0x%p\n", pjoin->exit_value);
              *pexit_value = pjoin->exit_value;
            }
        }
      else
        {
          sinfo("Thread is still running\n");

          /* Relinquish the data set semaphore.  Since pre-emption is
           * disabled, we can be certain that no task has the
           * opportunity to run between the time we relinquish the
           * join semaphore and the time that we wait on the thread exit
           * semaphore.
           */

          pthread_sem_give(&group->tg_joinsem);

          /* Take the thread's thread exit semaphore.  We will sleep here
           * until the thread exits.  We need to exercise caution because
           * there could be multiple threads waiting here for the same
           * pthread to exit.
           */

          pthread_sem_take(&pjoin->exit_sem, NULL, false);

          /* The thread has exited! Get the thread exit value */

          if (pexit_value)
            {
              *pexit_value = pjoin->exit_value;
              sinfo("exit_value=0x%p\n", pjoin->exit_value);
            }

          /* Post the thread's data semaphore so that the exiting thread
           * will know that we have received the data.
           */

          pthread_sem_give(&pjoin->data_sem);

          /* Retake the join semaphore, we need to hold this when
           * pthread_destroyjoin is called.
           */

          pthread_sem_take(&group->tg_joinsem, NULL, false);
        }

      /* Pre-emption is okay now. The logic still cannot be re-entered
       * because we hold the join semaphore
       */

      sched_unlock();

      /* Release our reference to the join structure and, if the reference
       * count decrements to zero, deallocate the join structure.
       */

      if (--pjoin->crefs <= 0)
        {
          pthread_destroyjoin(group, pjoin);
        }

      pthread_sem_give(&group->tg_joinsem);
      ret = OK;
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
