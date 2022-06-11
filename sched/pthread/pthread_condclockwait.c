/****************************************************************************
 * sched/pthread/pthread_condclockwait.c
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
  irqstate_t flags;
  int mypid = getpid();
  int ret = OK;
  int status;

  sinfo("cond=0x%p mutex=0x%p abstime=0x%p\n", cond, mutex, abstime);

  /* pthread_cond_clockwait() is a cancellation point */

  enter_cancellation_point();

  /* Make sure that non-NULL references were provided. */

  if (!cond || !mutex)
    {
      ret = EINVAL;
    }

  /* Make sure that the caller holds the mutex */

  else if (mutex->pid != mypid)
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
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      uint8_t mflags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      uint8_t type;
      int16_t nlocks;
#endif

      sinfo("Give up mutex...\n");

      /* We must disable pre-emption and interrupts here so that
       * the time stays valid until the wait begins.   This adds
       * complexity because we assure that interrupts and
       * pre-emption are re-enabled correctly.
       */

      sched_lock();
      flags = enter_critical_section();

      /* Give up the mutex */

      mutex->pid = INVALID_PROCESS_ID;
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      mflags     = mutex->flags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      type       = mutex->type;
      nlocks     = mutex->nlocks;
#endif
      ret        = pthread_mutex_give(mutex);
      if (ret == 0)
        {
          status = nxsem_clockwait_uninterruptible(
                   &cond->sem, clockid, abstime);
          if (status < 0)
            {
              ret = -status;
            }
        }

      /* Restore interrupts  (pre-emption will be enabled
       * when we fall through the if/then/else)
       */

      leave_critical_section(flags);

      /* Reacquire the mutex (retaining the ret). */

      sinfo("Re-locking...\n");

      status = pthread_mutex_take(mutex, NULL, false);
      if (status == OK)
        {
          mutex->pid    = mypid;
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
          mutex->flags  = mflags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          mutex->type   = type;
          mutex->nlocks = nlocks;
#endif
        }
      else if (ret == 0)
        {
          ret           = status;
        }

      /* Re-enable pre-emption (It is expected that interrupts
       * have already been re-enabled in the above logic)
       */

      sched_unlock();
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
