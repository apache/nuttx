/****************************************************************************
 * sched/pthread/pthread_condwait.c
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
  irqstate_t flags;

  sinfo("cond=%p mutex=%p\n", cond, mutex);

  /* pthread_cond_wait() is a cancellation point */

  enter_cancellation_point();

  /* Make sure that non-NULL references were provided. */

  if (cond == NULL || mutex == NULL)
    {
      ret = EINVAL;
    }

  /* Make sure that the caller holds the mutex */

  else if (mutex->pid != nxsched_gettid())
    {
      ret = EPERM;
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

      /* Give up the mutex */

      sinfo("Give up mutex / take cond\n");

      flags = enter_critical_section();
      mutex->pid = INVALID_PROCESS_ID;
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      mflags     = mutex->flags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      type       = mutex->type;
      nlocks     = mutex->nlocks;
#endif
      ret        = pthread_mutex_give(mutex);

      /* Take the semaphore.  This may be awakened only be a signal (EINTR)
       * or if the thread is canceled (ECANCELED)
       */

      status = pthread_sem_take(&cond->sem, NULL);
      if (ret == OK)
        {
          /* Report the first failure that occurs */

          ret = status;
        }

      leave_critical_section(flags);

      /* Reacquire the mutex.
       *
       * When cancellation points are enabled, we need to hold the mutex
       * when the pthread is canceled and cleanup handlers, if any, are
       * entered.
       */

      sinfo("Reacquire mutex...\n");

      status = pthread_mutex_take(mutex, NULL);
      if (ret == OK)
        {
          /* Report the first failure that occurs */

          ret = status;
        }

      /* Did we get the mutex? */

      if (status == OK)
        {
          /* Yes.. Then initialize it properly */

          mutex->pid    = nxsched_gettid();
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
          mutex->flags  = mflags;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          mutex->type   = type;
          mutex->nlocks = nlocks;
#endif
        }
    }

  leave_cancellation_point();
  sinfo("Returning %d\n", ret);
  return ret;
}
