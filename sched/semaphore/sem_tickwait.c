/****************************************************************************
 * sched/semaphore/sem_tickwait.c
 *
 *   Copyright (C) 2015-2017 Gregory Nutt. All rights reserved.
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
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_tickwait
 *
 * Description:
 *   This function is a lighter weight version of sem_timedwait().  It is
 *   non-standard and intended only for use within the RTOS.
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   start   - The system time that the delay is relative to.  If the
 *             current time is not the same as the start time, then the
 *             delay will be adjust so that the end time will be the same
 *             in any event.
 *   delay   - Ticks to wait from the start time until the semaphore is
 *             posted.  If ticks is zero, then this function is equivalent
 *             to nxsem_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   -ETIMEDOUT is returned on the timeout condition.
 *
 ****************************************************************************/

int nxsem_tickwait(FAR sem_t *sem, clock_t start, uint32_t delay)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  clock_t elapsed;
  int ret;

  DEBUGASSERT(sem != NULL && up_interrupt_context() == false &&
              rtcb->waitdog == NULL);

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

  if (delay == 0)
    {
      /* Return the errno from nxsem_trywait() */

      goto errout_with_irqdisabled;
    }

  /* Adjust the delay for any time since the delay was calculated */

  elapsed = clock_systimer() - start;
  if (/* elapsed >= (UINT32_MAX / 2) || */ elapsed >= delay)
    {
      ret = -ETIMEDOUT;
      goto errout_with_irqdisabled;
    }

  delay -= elapsed;

  /* Start the watchdog with interrupts still disabled */

  (void)wd_start(rtcb->waitdog, delay, (wdentry_t)nxsem_timeout,
                 1, getpid());

  /* Now perform the blocking wait */

  ret = nxsem_wait(sem);
  if (ret < 0)
    {
      goto errout_with_irqdisabled;
    }

  /* Stop the watchdog timer */

  wd_cancel(rtcb->waitdog);

  /* We can now restore interrupts and delete the watchdog */

  /* Success exits */

success_with_irqdisabled:

  /* Error exits */

errout_with_irqdisabled:
  leave_critical_section(flags);
  wd_delete(rtcb->waitdog);
  rtcb->waitdog = NULL;
  return ret;
}
