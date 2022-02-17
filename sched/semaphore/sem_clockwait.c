/****************************************************************************
 * sched/semaphore/sem_clockwait.c
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

#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "clock/clock.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_clockwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   clockid - The timing source to use in the conversion
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   That may be one of:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *
 ****************************************************************************/

int nxsem_clockwait(FAR sem_t *sem, clockid_t clockid,
                    FAR const struct timespec *abstime)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  sclock_t ticks;
  int status;
  int ret = ERROR;

  DEBUGASSERT(up_interrupt_context() == false);

  /* Verify the input parameters and, in case of an error, set
   * errno appropriately.
   */

#ifdef CONFIG_DEBUG_FEATURES
  if (!abstime || !sem)
    {
      return -EINVAL;
    }
#endif

  /* We will disable interrupts until we have completed the semaphore
   * wait.  We need to do this (as opposed to just disabling pre-emption)
   * because there could be interrupt handlers that are asynchronously
   * posting semaphores and to prevent race conditions with watchdog
   * timeout.  This is not too bad because interrupts will be re-
   * enabled while we are blocked waiting for the semaphore.
   */

  flags = enter_critical_section();

  /* Try to take the semaphore without waiting. */

  ret = nxsem_trywait(sem);
  if (ret == OK)
    {
      /* We got it! */

      goto out;
    }

  /* We will have to wait for the semaphore.  Make sure that we were provided
   * with a valid timeout.
   */

  if (abstime->tv_nsec < 0 || abstime->tv_nsec >= 1000000000)
    {
      ret = -EINVAL;
      goto out;
    }

  /* Convert the timespec to clock ticks.  We must have interrupts
   * disabled here so that this time stays valid until the wait begins.
   *
   * clock_abstime2ticks() returns zero on success or a POSITIVE errno
   * value on failure.
   */

  status = clock_abstime2ticks(clockid, abstime, &ticks);

  /* If the time has already expired return immediately. */

  if (status == OK && ticks <= 0)
    {
      ret = -ETIMEDOUT;
      goto out;
    }

  /* Handle any time-related errors */

  if (status != OK)
    {
      ret = -status;
      goto out;
    }

  /* Start the watchdog */

  wd_start(&rtcb->waitdog, ticks, nxsem_timeout, getpid());

  /* Now perform the blocking wait.  If nxsem_wait() fails, the
   * negated errno value will be returned below.
   */

  ret = nxsem_wait(sem);

  /* Stop the watchdog timer */

  wd_cancel(&rtcb->waitdog);

  /* We can now restore interrupts and delete the watchdog */

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxsem_clockwait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_timedwait(), which is
 *   uninterruptible and convenient for use.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   clockid - The timing source to use in the conversion
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_clockwait_uninterruptible(FAR sem_t *sem, clockid_t clockid,
                                    FAR const struct timespec *abstime)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_clockwait(sem, clockid, abstime);
    }
  while (ret == -EINTR);

  return ret;
}

/****************************************************************************
 * Name: sem_clockwait
 *
 * Description:
 *   This function will lock the semaphore referenced by sem as in the
 *   sem_wait() function. However, if the semaphore cannot be locked without
 *   waiting for another process or thread to unlock the semaphore by
 *   performing a sem_post() function, this wait will be terminated when the
 *   specified timeout expires.
 *
 *   The timeout will expire when the absolute time specified by abstime
 *   passes, as measured by the clock on which timeouts are based (that is,
 *   when the value of that clock equals or exceeds abstime), or if the
 *   absolute time specified by abstime has already been passed at the
 *   time of the call.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   clockid - The timing source to use in the conversion
 *   abstime - The absolute time to wait until a timeout is declared.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, -1 (ERROR) is returned
 *   and the errno is set appropriately:
 *
 *   EINVAL    The sem argument does not refer to a valid semaphore.  Or the
 *             thread would have blocked, and the abstime parameter specified
 *             a nanoseconds field value less than zero or greater than or
 *             equal to 1000 million.
 *   ETIMEDOUT The semaphore could not be locked before the specified timeout
 *             expired.
 *   EDEADLK   A deadlock condition was detected.
 *   EINTR     A signal interrupted this function.
 *   ECANCELED May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int sem_clockwait(FAR sem_t *sem, clockid_t clockid,
                  FAR const struct timespec *abstime)
{
  int ret;

  /* sem_timedwait() is a cancellation point */

  enter_cancellation_point();

  /* Let nxsem_timedout() do the work */

  ret = nxsem_clockwait(sem, clockid, abstime);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
