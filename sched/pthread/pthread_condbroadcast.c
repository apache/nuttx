/****************************************************************************
 * sched/pthread/pthread_condbroadcast.c
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
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/atomic.h>

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

  sinfo("cond=%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }
  else
    {
      int wcnt = atomic_read(COND_WAIT_COUNT(cond));

      /* Loop until all of the waiting threads have been restarted. */

      while (wcnt > 0)
        {
          if (atomic_cmpxchg(COND_WAIT_COUNT(cond), &wcnt, wcnt - 1))
            {
              /* Post the condition semaphore to wake up a waiting thread.
               * Only the highest priority waiting thread will get to execute
               */

              ret = -nxsem_post(&cond->sem);

              /* Decrement the waiter count */

              wcnt--;
            }
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
