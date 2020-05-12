/****************************************************************************
 * sched/pthread/pthread_mutextimedlock.c
 *
 *   Copyright (C) 2007-2009, 2017, 2019 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_timedlock
 *
 * Description:
 *   The pthread_mutex_timedlock() function will lock the mutex object
 *   referenced by mutex. If the mutex is already locked, the calling
 *   thread will block until the mutex becomes available as in the
 *   pthread_mutex_lock() function. If the mutex cannot be locked without
 *   waiting for another thread to unlock the mutex, this wait will be
 *   terminated when the specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by
 *   abs_timeout passes, as measured by the clock on which timeouts are
 *   based (that is, when the value of that clock equals or exceeds
 *   abs_timeout), or if the absolute time specified by abs_timeout
 *   has already been passed at the time of the call.
 *
 * Input Parameters:
 *   mutex - A reference to the mutex to be locked.
 *   abs_timeout - max wait time (NULL wait forever)
 *
 * Returned Value:
 *   0 on success or an errno value on failure.  Note that the errno EINTR
 *   is never returned by pthread_mutex_timedlock().
 *   errno is ETIMEDOUT if mutex could not be locked before the specified
 *   timeout expired
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   - This implementation does not return EAGAIN when the mutex could not be
 *     acquired because the maximum number of recursive locks for mutex has
 *     been exceeded.
 *
 ****************************************************************************/

int pthread_mutex_timedlock(FAR pthread_mutex_t *mutex,
                            FAR const struct timespec *abs_timeout)
{
  int mypid = (int)getpid();
  int ret = EINVAL;

  sinfo("mutex=0x%p\n", mutex);
  DEBUGASSERT(mutex != NULL);

  if (mutex != NULL)
    {
      /* Make sure the semaphore is stable while we make the following
       * checks.  This all needs to be one atomic action.
       */

      sched_lock();

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      /* All mutex types except for NORMAL (and DEFAULT) will return
       * and an error  error if the caller does not hold the mutex.
       */

      if (mutex->type != PTHREAD_MUTEX_NORMAL && mutex->pid == mypid)
        {
          /* Yes.. Is this a recursive mutex? */

          if (mutex->type == PTHREAD_MUTEX_RECURSIVE)
            {
              /* Yes... just increment the number of locks held and return
               * success.
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
            {
              /* No, then we would deadlock... return an error (default
               * behavior is like PTHREAD_MUTEX_ERRORCHECK)
               *
               * NOTE: This is the correct behavior for a 'robust', NORMAL
               * mutex.  Compliant behavior for non-robust mutex should not
               * include these checks.  In that case, it the deadlock
               * condition should not be detected and the thread should be
               * permitted to deadlock.
               */

              serr("ERROR: Returning EDEADLK\n");
              ret = EDEADLK;
            }
        }
      else
#endif /* CONFIG_PTHREAD_MUTEX_TYPES */

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      /* The calling thread does not hold the semaphore.  The correct
       * behavior for the 'robust' mutex is to verify that the holder of the
       * mutex is still valid.  This is protection from the case
       * where the holder of the mutex has exitted without unlocking it.
       */

#ifdef CONFIG_PTHREAD_MUTEX_BOTH
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      /* Include check if this is a NORMAL mutex and that it is robust */

      if (mutex->pid > 0 &&
          ((mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0 ||
           mutex->type != PTHREAD_MUTEX_NORMAL) &&
          nxsched_get_tcb(mutex->pid) == NULL)

#else /* CONFIG_PTHREAD_MUTEX_TYPES */
      /* This can only be a NORMAL mutex.  Include check if it is robust */

      if (mutex->pid > 0 &&
          (mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0 &&
          nxsched_get_tcb(mutex->pid) == NULL)

#endif /* CONFIG_PTHREAD_MUTEX_TYPES */
#else /* CONFIG_PTHREAD_MUTEX_ROBUST */
      /* This mutex is always robust, whatever type it is. */

      if (mutex->pid > 0 && nxsched_get_tcb(mutex->pid) == NULL)
#endif
        {
          DEBUGASSERT(mutex->pid != 0); /* < 0: available, >0 owned, ==0 error */
          DEBUGASSERT((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT) != 0);

          /* A thread holds the mutex, but there is no such thread.  POSIX
           * requires that the 'robust' mutex return EOWNERDEAD in this
           * case.  It is then the caller's responsibility to call
           * pthread_mutx_consistent() to fix the mutex.
           */

          mutex->flags |= _PTHREAD_MFLAGS_INCONSISTENT;
          ret           = EOWNERDEAD;
        }
      else
#endif /* !CONFIG_PTHREAD_MUTEX_UNSAFE */

        {
          /* Take the underlying semaphore, waiting if necessary.  NOTE that
           * is required to deadlock for the case of the non-robust NORMAL
           * or default mutex.
           */

          ret = pthread_mutex_take(mutex, abs_timeout, true);

          /* If we successfully obtained the semaphore, then indicate
           * that we own it.
           */

          if (ret == OK)
            {
              mutex->pid    = mypid;
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
              mutex->nlocks = 1;
#endif
            }
        }

      sched_unlock();
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
