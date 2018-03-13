/****************************************************************************
 * sched/pthread/pthread_completejoin.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

  status = nxsem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
  if (status == OK && ntasks_waiting < 0)
    {
      /* Set the data semaphore so that this thread will be
       * awakened when all waiting tasks receive the data
       */

      (void)nxsem_init(&pjoin->data_sem, 0, (ntasks_waiting + 1));

      /* Post the semaphore to restart each thread that is waiting
       * on the semaphore
       */

      do
        {
          status = pthread_sem_give(&pjoin->exit_sem);
          if (status == OK)
            {
              status = nxsem_getvalue(&pjoin->exit_sem, &ntasks_waiting);
            }
        }
      while (ntasks_waiting < 0 && status == OK);

      /* Now wait for all these restarted tasks to obtain the return
       * value.
       */

      (void)pthread_sem_take(&pjoin->data_sem, false);
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

  (void)pthread_sem_take(&group->tg_joinsem, false);
  pjoin = pthread_findjoininfo(group, pid);
  if (!pjoin)
    {
      serr("ERROR: Could not find join info, pid=%d\n", pid);
      (void)pthread_sem_give(&group->tg_joinsem);
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

      (void)pthread_sem_give(&group->tg_joinsem);
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

  (void)nxsem_destroy(&pjoin->data_sem);
  (void)nxsem_destroy(&pjoin->exit_sem);

  /* And deallocate the pjoin structure */

  sched_kfree(pjoin);
}

