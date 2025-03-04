/****************************************************************************
 * sched/pthread/pthread_mutexdestroy.c
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

#include <pthread.h>
#include <signal.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_destroy
 *
 * Description:
 *   Destroy a mutex.
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

int pthread_mutex_destroy(FAR pthread_mutex_t *mutex)
{
  int ret = EINVAL;
  int status;

  sinfo("mutex=%p\n", mutex);
  DEBUGASSERT(mutex != NULL);

  if (mutex != NULL)
    {
      pid_t pid;

      pid = mutex_get_holder(&mutex->mutex);

      /* Is the mutex available? */

      if (pid >= 0)
        {
          /* < 0: available, >0 owned, ==0 error */

          DEBUGASSERT(pid != 0);
          /* No.. Verify that the PID still exists.  We may be destroying
           * the mutex after cancelling a pthread and the mutex may have
           * been in a bad state owned by the dead pthread.  NOTE: The
           * following behavior is unspecified for pthread_mutex_destroy()
           * (see pthread_mutex_consistent()).
           *
           * If the holding thread is still valid, then we should be able to
           * map its PID to the underlying TCB. That is what
           * nxsched_get_tcb() does.
           */

          if (nxsched_get_tcb(pid) == NULL)
            {
              /* The thread associated with the PID no longer exists */

              /* Reset the semaphore.  If threads are were on this
               * semaphore, then this will awakened them and make
               * destruction of the semaphore impossible here.
               */

              mutex_reset(&mutex->mutex);

              /* Check if the reset caused some other thread to lock the
               * mutex.
               */

              if (mutex_is_locked(&mutex->mutex))
                {
                  /* Yes.. then we cannot destroy the mutex now. */

                  ret = EBUSY;
                }

              /* Destroy the underlying semaphore */

              else
                {
                  status = mutex_destroy(&mutex->mutex);
                  ret = (status < 0) ? -status : OK;
                }
            }
          else
            {
              ret = EBUSY;
            }
        }
      else
        {
          /* Destroy the semaphore
           *
           * REVISIT:  What if there are threads waiting on the semaphore?
           * Perhaps this logic should all nxsem_reset() first?
           */

          status = mutex_destroy(&mutex->mutex);
          ret = ((status < 0) ? -status : OK);
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
