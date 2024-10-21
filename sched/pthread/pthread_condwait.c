/****************************************************************************
 * sched/pthread/pthread_condwait.c
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

#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/cancelpt.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: int pthread_cond_wait
 *
 * Description:
 *   A thread can wait for a condition variable to be signalled or broadcast.
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

int pthread_cond_wait(FAR pthread_cond_t *cond, FAR pthread_mutex_t *mutex)
{
  int status;
  int ret;

  sinfo("cond=%p mutex=%p\n", cond, mutex);

  /* pthread_cond_wait() is a cancellation point */

  enter_cancellation_point();

  /* Make sure that non-NULL references were provided. */

  if (cond == NULL || mutex == NULL)
    {
      ret = EINVAL;
    }

  /* Make sure that the caller holds the mutex */

  else if (!mutex_is_hold(&mutex->mutex))
    {
      ret = EPERM;
    }
  else
    {
      unsigned int nlocks;

      /* Give up the mutex */

      sinfo("Give up mutex / take cond\n");

      cond->lock_count--;
      ret = pthread_mutex_breaklock(mutex, &nlocks);

      status = -nxsem_wait_uninterruptible(&cond->sem);
      if (ret == OK)
        {
          /* Report the first failure that occurs */

          ret = status;
        }

      /* Reacquire the mutex.
       *
       * When cancellation points are enabled, we need to hold the mutex
       * when the pthread is canceled and cleanup handlers, if any, are
       * entered.
       */

      sinfo("Reacquire mutex...\n");

      status = pthread_mutex_restorelock(mutex, nlocks);
      if (ret == OK)
        {
          /* Report the first failure that occurs */

          ret = status;
        }
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
