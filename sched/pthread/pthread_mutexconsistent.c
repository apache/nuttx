/****************************************************************************
 * sched/pthread/pthread_mutexconsistent.c
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

#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_consistent
 *
 * Description:
 *   If mutex is a robust mutex in an inconsistent state, the
 *   pthread_mutex_consistent() function can be used to mark the state
 *   protected by the mutex referenced by mutex as consistent again.
 *
 *   If an owner of a robust mutex terminates while holding the mutex, the
 *   mutex becomes inconsistent and the next thread that acquires the mutex
 *   lock will be notified of the state by the return value EOWNERDEAD.
 *   In this case, the mutex does not become normally usable again until the
 *   state is marked consistent.
 *
 *   If the thread which acquired the mutex lock with the return value
 *   EOWNERDEAD terminates before calling either pthread_mutex_consistent()
 *   or pthread_mutex_unlock(), the next thread that acquires the mutex lock
 *   will be notified about the state of the mutex by the return value
 *   EOWNERDEAD.
 *
 *   The behavior is undefined if the value specified by the mutex argument
 *   to pthread_mutex_consistent() does not refer to an initialized mutex.
 *
 * Input Parameters:
 *   mutex -- a reference to the mutex to be made consistent
 *
 * Returned Value:
 *   Upon successful completion, the pthread_mutex_consistent() function
 *   will return zero. Otherwise, an error value will be returned to
 *   indicate the error.
 *
 ****************************************************************************/

int pthread_mutex_consistent(FAR pthread_mutex_t *mutex)
{
  int ret = EINVAL;
  int status;

  sinfo("mutex=0x%p\n", mutex);
  DEBUGASSERT(mutex != NULL);

  if (mutex != NULL)
    {
      /* Make sure the mutex is stable while we make the following checks. */

      sched_lock();

      /* Is the mutex available? */

      DEBUGASSERT(mutex->pid != 0); /* < 0: available, >0 owned, ==0 error */
      if (mutex->pid >= 0)
        {
          /* No.. Verify that the thread associated with the PID still
           * exists.  We may be destroying the mutex after cancelling a
           * pthread and the mutex may have been in a bad state owned by
           * the dead pthread.  NOTE: The following is unspecified behavior
           * (see pthread_mutex_consistent()).
           *
           * If the holding thread is still valid, then we should be able to
           * map its PID to the underlying TCB.  That is what
           * nxsched_get_tcb() does.
           */

          if (nxsched_get_tcb(mutex->pid) == NULL)
            {
              /* The thread associated with the PID no longer exists */

              mutex->pid    = -1;
              mutex->flags &= _PTHREAD_MFLAGS_ROBUST;
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
              mutex->nlocks = 0;
#endif
              /* Reset the semaphore.  This has the same affect as if the
               * dead task had called pthread_mutex_unlock().
               */

              status = nxsem_reset((FAR sem_t *)&mutex->sem, 1);
              if (status < 0)
                {
                  ret = -status;
                }
              else
                {
                  ret = OK;
                }
            }

          /* Otherwise the mutex is held by some active thread.  Let's not
           * touch anything!
           */
        }
      else
        {
          /* There is no holder of the mutex.  Just make sure the
           * inconsistent flag is cleared and the number of locks is zero.
           */

          mutex->flags &= _PTHREAD_MFLAGS_ROBUST;
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          mutex->nlocks = 0;
#endif
          ret = OK;
        }

      sched_unlock();
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
