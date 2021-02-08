/****************************************************************************
 * sched/pthread/pthread_mutexunlock.c
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_islocked
 *
 * Description:
 *   Return true is the mutex is locked.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if the mutex is locked
 *
 ****************************************************************************/

static inline bool pthread_mutex_islocked(FAR struct pthread_mutex_s *mutex)
{
  int semcount = mutex->sem.semcount;

  /* The underlying semaphore should have a count less than 2:
   *
   *  1 == mutex is unlocked.
   *  0 == mutex is locked with no waiters
   * -n == mutex is locked with 'n' waiters.
   */

  DEBUGASSERT(semcount < 2);
  return semcount < 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_unlock
 *
 * Description:
 *   The pthread_mutex_unlock() function releases the mutex object referenced
 *   by mutex. The manner in which a mutex is released is dependent upon the
 *   mutex's type attribute. If there are threads blocked on the mutex object
 *   referenced by mutex when pthread_mutex_unlock() is called, resulting in
 *   the mutex becoming available, the scheduling policy is used to determine
 *   which thread shall acquire the mutex. (In the case of
 *   PTHREAD_MUTEX_RECURSIVE mutexes, the mutex becomes available when the
 *   count reaches zero and the calling thread no longer has any locks on
 *   this mutex).
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return
 *   from the signal handler the thread resumes waiting for the mutex as if
 *   it was not interrupted.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_unlock(FAR pthread_mutex_t *mutex)
{
  int ret = EPERM;

  sinfo("mutex=0x%p\n", mutex);
  DEBUGASSERT(mutex != NULL);
  if (mutex == NULL)
    {
      return EINVAL;
    }

  /* Make sure the semaphore is stable while we make the following checks.
   * This all needs to be one atomic action.
   */

  sched_lock();

  /* The unlock operation is only performed if the mutex is actually locked.
   * EPERM *must* be returned if the mutex type is PTHREAD_MUTEX_ERRORCHECK
   * or PTHREAD_MUTEX_RECURSIVE, or the mutex is a robust mutex, and the
   * current thread does not own the mutex.  Behavior is undefined for the
   * remaining case.
   */

  if (pthread_mutex_islocked(mutex))
    {
#if !defined(CONFIG_PTHREAD_MUTEX_UNSAFE) || defined(CONFIG_PTHREAD_MUTEX_TYPES)
      /* Does the calling thread own the semaphore?  If no, should we return
       * an error?
       *
       * Error checking is always performed for ERRORCHECK and RECURSIVE
       * mutex types.  Error checking is only performed for NORMAL (or
       * DEFAULT) mutex type if the NORMAL mutex is robust.  That is either:
       *
       *   1. CONFIG_PTHREAD_MUTEX_ROBUST is defined, or
       *   2. CONFIG_PTHREAD_MUTEX_BOTH is defined and the robust flag is set
       */

#if defined(CONFIG_PTHREAD_MUTEX_ROBUST)
      /* Not that error checking is always performed if the configuration has
       * CONFIG_PTHREAD_MUTEX_ROBUST defined.  Just check if the calling
       * thread owns the semaphore.
       */

      if (mutex->pid != (int)getpid())

#elif defined(CONFIG_PTHREAD_MUTEX_UNSAFE) && defined(CONFIG_PTHREAD_MUTEX_TYPES)
      /* If mutex types are not supported, then all mutexes are NORMAL (or
       * DEFAULT).  Error checking should never be performed for the
       * non-robust NORMAL mutex type.
       */

      if (mutex->type != PTHREAD_MUTEX_NORMAL && mutex->pid != (int)getpid())

#else /* CONFIG_PTHREAD_MUTEX_BOTH */
      /* Skip the error check if this is a non-robust NORMAL mutex */

      bool errcheck = ((mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0);
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      errcheck     |= (mutex->type != PTHREAD_MUTEX_NORMAL);
#endif

      /* Does the calling thread own the semaphore?  If not should we report
       * the EPERM error?
       */

      if (errcheck && mutex->pid != (int)getpid())
#endif
        {
          /* No... return an EPERM error.
           *
           * Per POSIX:  "EPERM should be returned if the mutex type is
           * PTHREAD_MUTEX_ERRORCHECK or PTHREAD_MUTEX_RECURSIVE, or the
           * mutex is a robust mutex, and the current thread does not own
           * the mutex."
           *
           * For the case of the non-robust PTHREAD_MUTEX_NORMAL mutex,
           * the behavior is undefined.
           */

          serr("ERROR: Holder=%d returning EPERM\n", mutex->pid);
          ret = EPERM;
        }
      else
#endif /* !CONFIG_PTHREAD_MUTEX_UNSAFE || CONFIG_PTHREAD_MUTEX_TYPES */

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      /* Yes, the caller owns the semaphore.. Is this a recursive mutex? */

      if (mutex->type == PTHREAD_MUTEX_RECURSIVE && mutex->nlocks > 1)
        {
          /* This is a recursive mutex and we there are multiple locks held.
           * Retain the mutex lock, just decrement the count of locks held,
           * and return success.
           */

          mutex->nlocks--;
          ret = OK;
        }
      else

#endif /* CONFIG_PTHREAD_MUTEX_TYPES */

      /* This is either a non-recursive mutex or is the outermost unlock of
       * a recursive mutex.
       *
       * In the case where the calling thread is NOT the holder of the
       * thread, the behavior is undefined per POSIX.
       * Here we do the same as GLIBC:
       * We allow the other thread to release the mutex even though it does
       * not own it.
       */

        {
          /* Nullify the pid and lock count then post the semaphore */

          mutex->pid    = -1;
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          mutex->nlocks = 0;
#endif
          ret = pthread_mutex_give(mutex);
        }
    }

  sched_unlock();
  sinfo("Returning %d\n", ret);
  return ret;
}
