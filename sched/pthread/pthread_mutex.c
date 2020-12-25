/****************************************************************************
 * sched/pthread/pthread_mutex.c
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_add
 *
 * Description:
 *   Add the mutex to the list of mutexes held by this pthread.
 *
 * Input Parameters:
 *  mutex - The mutex to be locked
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pthread_mutex_add(FAR struct pthread_mutex_s *mutex)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;

  DEBUGASSERT(mutex->flink == NULL);

  /* Add the mutex to the list of mutexes held by this pthread */

  flags        = enter_critical_section();
  mutex->flink = rtcb->mhead;
  rtcb->mhead  = mutex;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: pthread_mutex_remove
 *
 * Description:
 *   Remove the mutex to the list of mutexes held by this pthread.
 *
 * Input Parameters:
 *  mutex - The mutex to be locked
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pthread_mutex_remove(FAR struct pthread_mutex_s *mutex)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct pthread_mutex_s *curr;
  FAR struct pthread_mutex_s *prev;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Remove the mutex from the list of mutexes held by this task */

  for (prev = NULL, curr = rtcb->mhead;
       curr != NULL && curr != mutex;
       prev = curr, curr = curr->flink)
    {
    }

  DEBUGASSERT(curr == mutex);

  /* Remove the mutex from the list.  prev == NULL means that the mutex
   * to be removed is at the head of the list.
   */

  if (prev == NULL)
    {
      rtcb->mhead = mutex->flink;
    }
  else
    {
      prev->flink = mutex->flink;
    }

  mutex->flink = NULL;
  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_take
 *
 * Description:
 *   Take the pthread_mutex, waiting if necessary.  If successful, add the
 *   mutex to the list of mutexes held by this thread.
 *
 * Input Parameters:
 *  mutex - The mutex to be locked
 *  intr  - false: ignore EINTR errors when locking; true treat EINTR as
 *          other errors by returning the errno value
 *
 * Returned Value:
 *   0 on success or an errno value on failure.
 *
 ****************************************************************************/

int pthread_mutex_take(FAR struct pthread_mutex_s *mutex,
                       FAR const struct timespec *abs_timeout, bool intr)
{
  int ret = EINVAL;

  /* Verify input parameters */

  DEBUGASSERT(mutex != NULL);
  if (mutex != NULL)
    {
      /* Make sure that no unexpected context switches occur */

      sched_lock();

      /* Error out if the mutex is already in an inconsistent state. */

      if ((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT) != 0)
        {
          ret = EOWNERDEAD;
        }
      else
        {
          /* Take semaphore underlying the mutex.  pthread_sem_take
           * returns zero on success and a positive errno value on failure.
           */

          ret = pthread_sem_take(&mutex->sem, abs_timeout, intr);
          if (ret == OK)
            {
              /* Check if the holder of the mutex has terminated without
               * releasing.  In that case, the state of the mutex is
               * inconsistent and we return EOWNERDEAD.
               */

              if ((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT) != 0)
                {
                  ret = EOWNERDEAD;
                }

              /* Add the mutex to the list of mutexes held by this task */

              else
                {
                  pthread_mutex_add(mutex);
                }
            }
        }

      sched_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: pthread_mutex_trytake
 *
 * Description:
 *   Try to take the pthread_mutex without waiting.  If successful, add the
 *   mutex to the list of mutexes held by this thread.
 *
 * Input Parameters:
 *  mutex - The mutex to be locked
 *  intr  - false: ignore EINTR errors when locking; true treat EINTR as
 *          other errors by returning the errno value
 *
 * Returned Value:
 *   0 on success or an errno value on failure.
 *
 ****************************************************************************/

int pthread_mutex_trytake(FAR struct pthread_mutex_s *mutex)
{
  int ret = EINVAL;

  /* Verify input parameters */

  DEBUGASSERT(mutex != NULL);
  if (mutex != NULL)
    {
      /* Make sure that no unexpected context switches occur */

      sched_lock();

      /* Error out if the mutex is already in an inconsistent state. */

      if ((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT) != 0)
        {
          ret = EOWNERDEAD;
        }
      else
        {
          /* Try to take the semaphore underlying the mutex */

          ret = nxsem_trywait(&mutex->sem);
          if (ret < 0)
            {
              ret = -ret;
            }
          else
            {
              /* Add the mutex to the list of mutexes held by this task */

              pthread_mutex_add(mutex);
            }
        }

      sched_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: pthread_mutex_give
 *
 * Description:
 *   Take the pthread_mutex and, if successful, add the mutex to the list of
 *   mutexes held by this thread.
 *
 * Input Parameters:
 *  mutex - The mutex to be unlocked
 *
 * Returned Value:
 *   0 on success or an errno value on failure.
 *
 ****************************************************************************/

int pthread_mutex_give(FAR struct pthread_mutex_s *mutex)
{
  int ret = EINVAL;

  /* Verify input parameters */

  DEBUGASSERT(mutex != NULL);
  if (mutex != NULL)
    {
      /* Remove the mutex from the list of mutexes held by this task */

      pthread_mutex_remove(mutex);

      /* Now release the underlying semaphore */

      ret = pthread_sem_give(&mutex->sem);
    }

  return ret;
}
