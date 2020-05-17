/****************************************************************************
 * sched/pthread/pthread_condsignal.c
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
#include <errno.h>
#include <debug.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_signal
 *
 * Description:
 *    A thread can signal on a condition variable.
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

int pthread_cond_signal(FAR pthread_cond_t *cond)
{
  int ret = OK;
  int sval;

  sinfo("cond=0x%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }
  else
    {
      /* Get the current value of the semaphore */

      if (nxsem_get_value((FAR sem_t *)&cond->sem, &sval) != OK)
        {
          ret = EINVAL;
        }

      /* If the value is less than zero (meaning that one or more
       * thread is waiting), then post the condition semaphore.
       * Only the highest priority waiting thread will get to execute
       */

      else
        {
          /* One of my objectives in this design was to make
           * pthread_cond_signal() usable from interrupt handlers.  However,
           * from interrupt handlers, you cannot take the associated mutex
           * before signaling the condition.  As a result, I think that
           * there could be a race condition with the following logic which
           * assumes that the if sval < 0 then the thread is waiting.
           * Without the mutex, there is no atomic, protected operation that
           * will guarantee this to be so.
           */

          sinfo("sval=%d\n", sval);
          if (sval < 0)
            {
              sinfo("Signalling...\n");
              ret = pthread_sem_give((FAR sem_t *)&cond->sem);
            }
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
