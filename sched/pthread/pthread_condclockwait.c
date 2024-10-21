/****************************************************************************
 * sched/pthread/pthread_condclockwait.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "pthread/pthread.h"
#include "clock/clock.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_clockwait
 *
 * Description:
 *   A thread can perform a timed wait on a condition variable.
 *
 * Input Parameters:
 *   cond    - the condition variable to wait on
 *   mutex   - the mutex that protects the condition variable
 *   clockid - The timing source to use in the conversion
 *   abstime - wait until this absolute time
 *
 * Returned Value:
 *   OK (0) on success; A non-zero errno value is returned on failure.
 *
 * Assumptions:
 *   Timing is of resolution 1 msec, with +/-1 millisecond accuracy.
 *
 ****************************************************************************/

int pthread_cond_clockwait(FAR pthread_cond_t *cond,
                           FAR pthread_mutex_t *mutex,
                           clockid_t clockid,
                           FAR const struct timespec *abstime)
{
  int ret = OK;
  int status;

  sinfo("cond=%p mutex=%p abstime=%p\n", cond, mutex, abstime);

  /* pthread_cond_clockwait() is a cancellation point */

  enter_cancellation_point();

  /* Make sure that non-NULL references were provided. */

  if (!cond || !mutex)
    {
      ret = EINVAL;
    }

  /* Make sure that the caller holds the mutex */

  else if (!mutex_is_hold(&mutex->mutex))
    {
      ret = EPERM;
    }

  /* If no wait time is provided, this function degenerates to
   * the same behavior as pthread_cond_wait().
   */

  else if (!abstime)
    {
      ret = pthread_cond_wait(cond, mutex);
    }

  else
    {
      unsigned int nlocks;

      sinfo("Give up mutex...\n");

      cond->lock_count--;

      /* Give up the mutex */

      ret = pthread_mutex_breaklock(mutex, &nlocks);
      if (ret == 0)
        {
          status = nxsem_clockwait_uninterruptible(&cond->sem,
                                                   clockid, abstime);
          if (status < 0)
            {
              ret = -status;
            }
        }

      /* Reacquire the mutex (retaining the ret). */

      sinfo("Re-locking...\n");

      status = pthread_mutex_restorelock(mutex, nlocks);
      if (ret == 0)
        {
          ret = status;
        }
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
