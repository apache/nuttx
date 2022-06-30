/****************************************************************************
 * sched/pthread/pthread_mutextrylock.c
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

#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_trylock
 *
 * Description:
 *   The function pthread_mutex_trylock() is identical to
 *   pthread_mutex_lock() except that if the mutex object referenced by the
 *   mutex is currently locked (by any thread, including the current
 *   thread), the call returns immediately with the errno EBUSY.
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return
 *   from the signal handler the thread resumes waiting for the mutex as if
 *   it was not interrupted.
 *
 * Input Parameters:
 *   mutex - A reference to the mutex to be locked.
 *
 * Returned Value:
 *   0 on success or an errno value on failure.  Note that the errno EINTR
 *   is never returned by pthread_mutex_trylock().
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - This implementation does not return EAGAIN when the mutex could not be
 *     acquired because the maximum number of recursive locks for mutex has
 *     been exceeded.
 *
 ****************************************************************************/

int pthread_mutex_trylock(FAR pthread_mutex_t *mutex)
{
  int status;
  int ret = EINVAL;

  sinfo("mutex=%p\n", mutex);
  DEBUGASSERT(mutex != NULL);

  if (mutex != NULL)
    {
      pid_t mypid = getpid();

      /* Make sure the semaphore is stable while we make the following
       * checks.  This all needs to be one atomic action.
       */

      sched_lock();

      /* Try to get the semaphore. */

      status = pthread_mutex_trytake(mutex);
      if (status == OK)
        {
          /* If we successfully obtained the semaphore, then indicate
           * that we own it.
           */

          mutex->pid = mypid;

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          if (mutex->type == PTHREAD_MUTEX_RECURSIVE)
            {
              mutex->nlocks = 1;
            }
#endif

          ret = OK;
        }

      /* pthread_mutex_trytake failed.  Did it fail because the semaphore
       * was not available?
       */

      else if (status == EAGAIN)
        {
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          /* Check if recursive mutex was locked by the calling thread. */

          if (mutex->type == PTHREAD_MUTEX_RECURSIVE && mutex->pid == mypid)
            {
              /* Increment the number of locks held and return
               * successfully.
               */

              if (mutex->nlocks < INT16_MAX)
                {
                  mutex->nlocks++;
                  ret = OK;
                }
              else
                {
                  ret = EOVERFLOW;
                }
            }
          else
#endif

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
          /* The calling thread does not hold the semaphore.  The correct
           * behavior for the 'robust' mutex is to verify that the holder of
           * the mutex is still valid.  This is protection from the case
           * where the holder of the mutex has exitted without unlocking it.
           */

#ifdef CONFIG_PTHREAD_MUTEX_BOTH
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          /* Check if this NORMAL mutex is robust */

          if (mutex->pid > 0 &&
              ((mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0 ||
               mutex->type != PTHREAD_MUTEX_NORMAL) &&
              nxsched_get_tcb(mutex->pid) == NULL)

#else /* CONFIG_PTHREAD_MUTEX_TYPES */
          /* Check if this NORMAL mutex is robust */

          if (mutex->pid > 0 &&
              (mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0 &&
              nxsched_get_tcb(mutex->pid) == NULL)

#endif /* CONFIG_PTHREAD_MUTEX_TYPES */
#else /* CONFIG_PTHREAD_MUTEX_ROBUST */
          /* This mutex is always robust, whatever type it is. */

          if (mutex->pid > 0 && nxsched_get_tcb(mutex->pid) == NULL)
#endif
            {
              /* < 0: available, >0 owned, ==0 error */

              DEBUGASSERT(mutex->pid != 0);
              DEBUGASSERT((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT)
                          != 0);

              /* A thread holds the mutex, but there is no such thread.
               * POSIX requires that the 'robust' mutex return EOWNERDEAD
               * in this case. It is then the caller's responsibility to
               * call pthread_mutex_consistent() to fix the mutex.
               */

              mutex->flags |= _PTHREAD_MFLAGS_INCONSISTENT;
              ret           = EOWNERDEAD;
            }

          /* The mutex is locked by another, active thread */

          else
#endif /* CONFIG_PTHREAD_MUTEX_UNSAFE */
            {
              ret = EBUSY;
            }
        }

      /* Some other, unhandled error occurred */

      else
        {
          ret = status;
        }

      sched_unlock();
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
