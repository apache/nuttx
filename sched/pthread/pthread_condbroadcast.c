/****************************************************************************
 * sched/pthread/pthread_condbroadcast.c
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
#include <errno.h>
#include <debug.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_broadcast
 *
 * Description:
 *    A thread broadcast on a condition variable.
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

int pthread_cond_broadcast(FAR pthread_cond_t *cond)
{
  int ret = OK;
  int sval;

  sinfo("cond=%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }
  else
    {
      /* Disable pre-emption until all of the waiting threads have been
       * restarted. This is necessary to assure that the sval behaves as
       * expected in the following while loop
       */

      sched_lock();

      /* Get the current value of the semaphore */

      if (nxsem_get_value((FAR sem_t *)&cond->sem, &sval) != OK)
        {
          ret = EINVAL;
        }
      else
        {
          /* Loop until all of the waiting threads have been restarted. */

          while (sval < 0)
            {
              /* If the value is less than zero (meaning that one or more
               * thread is waiting), then post the condition semaphore.
               * Only the highest priority waiting thread will get to execute
               */

              ret = pthread_sem_give((FAR sem_t *)&cond->sem);

              /* Increment the semaphore count (as was done by the
               * above post).
               */

              sval++;
            }
        }

      /* Now we can let the restarted threads run */

      sched_unlock();
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
