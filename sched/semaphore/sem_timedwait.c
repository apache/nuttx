/****************************************************************************
 * sched/semaphore/sem_timedwait.c
 *
 *   Copyright (C) 2011, 2013-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/cancelpt.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "clock/clock.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_timedwait
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
 *   - It is not a cancellaction point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem     - Semaphore object
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

int nxsem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  sclock_t ticks;
  int status;
  int ret = ERROR;

  DEBUGASSERT(up_interrupt_context() == false && rtcb->waitdog == NULL);

  /* Verify the input parameters and, in case of an error, set
   * errno appropriately.
   */

#ifdef CONFIG_DEBUG_FEATURES
  if (!abstime || !sem)
    {
      return -EINVAL;
    }
#endif

  /* Create a watchdog.  We will not actually need this watchdog
   * unless the semaphore is unavailable, but we will reserve it up
   * front before we enter the following critical section.
   */

  rtcb->waitdog = wd_create();
  if (!rtcb->waitdog)
    {
      return -ENOMEM;
    }

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

      goto success_with_irqdisabled;
    }

  /* We will have to wait for the semaphore.  Make sure that we were provided
   * with a valid timeout.
   */

  if (abstime->tv_nsec < 0 || abstime->tv_nsec >= 1000000000)
    {
      ret = -EINVAL;
      goto errout_with_irqdisabled;
    }

  /* Convert the timespec to clock ticks.  We must have interrupts
   * disabled here so that this time stays valid until the wait begins.
   *
   * clock_abstime2ticks() returns zero on success or a POSITIVE errno
   * value on failure.
   */

  status = clock_abstime2ticks(CLOCK_REALTIME, abstime, &ticks);

  /* If the time has already expired return immediately. */

  if (status == OK && ticks <= 0)
    {
      ret = -ETIMEDOUT;
      goto errout_with_irqdisabled;
    }

  /* Handle any time-related errors */

  if (status != OK)
    {
      ret = -status;
      goto errout_with_irqdisabled;
    }

  /* Start the watchdog */

  (void)wd_start(rtcb->waitdog, ticks, (wdentry_t)nxsem_timeout,
                 1, getpid());

  /* Now perform the blocking wait.  If nxsem_wait() fails, the
   * negated errno value will be returned below.
   */

  ret = nxsem_wait(sem);

  /* Stop the watchdog timer */

  wd_cancel(rtcb->waitdog);

  /* We can now restore interrupts and delete the watchdog */

success_with_irqdisabled:
errout_with_irqdisabled:
  leave_critical_section(flags);
  wd_delete(rtcb->waitdog);
  rtcb->waitdog = NULL;
  return ret;
}

/****************************************************************************
 * Name: sem_timedwait
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
 *
 ****************************************************************************/

int sem_timedwait(FAR sem_t *sem, FAR const struct timespec *abstime)
{
  int ret;

  /* sem_timedwait() is a cancellation point */

  (void)enter_cancellation_point();

  /* Let nxsem_timedout() do the work */

  ret = nxsem_timedwait(sem, abstime);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
