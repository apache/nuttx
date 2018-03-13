/****************************************************************************
 * sched/pthread/pthread_mutextrylock.c
 *
 *   Copyright (C) 2007-2009, 2017 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
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
 *   The function pthread_mutex_trylock() is identical to pthread_mutex_lock()
 *   except that if the mutex object referenced by mutex is currently locked
 *   (by any thread, including the current thread), the call returns immediately
 *   with the errno EBUSY.
 *
 *   If a signal is delivered to a thread waiting for a mutex, upon return from
 *   the signal handler the thread resumes waiting for the mutex as if it was
 *   not interrupted.
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

  sinfo("mutex=0x%p\n", mutex);
  DEBUGASSERT(mutex != NULL);

  if (mutex != NULL)
    {
      int mypid = (int)getpid();

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
       * was not avaialable?
       */

      else if (status == EAGAIN)
        {
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          /* Check if recursive mutex was locked by the calling thread. */

          if (mutex->type == PTHREAD_MUTEX_RECURSIVE && mutex->pid == mypid)
            {
              /* Increment the number of locks held and return successfully. */

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
                sched_gettcb(mutex->pid) == NULL)

#else /* CONFIG_PTHREAD_MUTEX_TYPES */
            /* Check if this NORMAL mutex is robust */

            if (mutex->pid > 0 &&
                (mutex->flags & _PTHREAD_MFLAGS_ROBUST) != 0 &&
                sched_gettcb(mutex->pid) == NULL)

#endif /* CONFIG_PTHREAD_MUTEX_TYPES */
#else /* CONFIG_PTHREAD_MUTEX_ROBUST */
            /* This mutex is always robust, whatever type it is. */

            if (mutex->pid > 0 && sched_gettcb(mutex->pid) == NULL)
#endif
              {
                DEBUGASSERT(mutex->pid != 0); /* < 0: available, >0 owned, ==0 error */
                DEBUGASSERT((mutex->flags & _PTHREAD_MFLAGS_INCONSISTENT) != 0);

                /* A thread holds the mutex, but there is no such thread.
                 * POSIX requires that the 'robust' mutex return EOWNERDEAD
                 * in this case. It is then the caller's responsibility to
                 * call pthread_mutx_consistent() fo fix the mutex.
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



