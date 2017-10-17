/****************************************************************************
 * arch/arm/src/bcm2708/bcm_tickless.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void arm_timer_initialize(void): Initializes the timer facilities.
 *     Called early in the initialization sequence (by up_intialize()).
 *   int up_timer_gettime(FAR struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(FAR const struct timespec *ts): Start (or re-starts)
 *     the interval timer.
 *
 * The RTOS will provide the following interfaces for use by the platform-
 * specific interval timer implementation:
 *
 *   void sched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 *
 ****************************************************************************/
/****************************************************************************
 * BCM2708 Timer Usage
 *
 * This implementation uses the BCM2708 64-bit system timer.  The 64-bit
 * timer provides the running time;  comparison registers are used to
 * generate interval interrupt.
 *
 * Comparison registrs C1 and C3 are available to the ARM.  C0 and C2 are
 * used by the GPU.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "up_arch.h"

#include "chip/bcm2708_systimr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
#  error CONFIG_SCHED_TICKLESS is required
#endif

#if CONFIG_USEC_PER_TICK != 1
#  error CONFIG_USEC_PER_TICK=1 is required
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct systimr_s
{
  volatile uint64_t start;     /* Timer interval timer started */
  volatile uint64_t interval;  /* Duration of the interval timer */
  volatile bool     running;   /* True if the interval timer is running */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct systimr_s g_systimr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm_read_systimr
 *
 * Description:
 *   Read the 64-bit system timer
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The 64-bit system timer
 *
 * Assumptions:
 *   Called from a critical section.
 *
 ****************************************************************************/

static uint64_t bcm_read_systimr(void)
{
  uint32_t check;
  uint32_t chi;
  uint32_t clo;

  /* Loop to handle overflow from CLO to CHI (should be a rare event) */

  do
    {
      chi   = getreg32(BCM_SYSTIMR_CHI);
      clo   = getreg32(BCM_SYSTIMR_CLO);
      check = getreg32(BCM_SYSTIMR_CHI);
    }
  while (chi != check);

  /* Then return the full 64-bit value */

  return ((uint64_t)chi << 32) | (uint64_t)clo;
}

/****************************************************************************
 * Name: bcm_convert_systimr
 *
 * Description:
 *   Convert the 64-bit system timer value to a standard struct ts
 *
 * Input Parameters:
 *   ts = Location to return the converted system timer value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm_convert_systimr(uint64_t usec, FAR struct timespec *ts)
{
  uint32_t sec;

  /* Convert and return the timer value */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;
}

/****************************************************************************
 * Name: bcm_systimr_interrupt
 *
 * Description:
 *   System tmer compare interrupt
 *
 * Input Parameters:
 *   Standard interrupt input parameters
 *
 * Returned Value:
 *   Always returns OK
 *
 * Assumptions:
 *   Interrupts ar disabled
 *
 ****************************************************************************/

static int bcm_systimr_interrupt(int irq, FAR void *context, FAR void *arg)
{
  /* Disable the Match 0 compare interrupt now. */

  up_disable_irq(BCM_IRQ_TIMER1);

  /* Clear the pending Match 0 compare interrupt */

  putreg32(SYSTIMR_C_M0, BCM_SYSTIMR_C);

  g_systimr.running  = false;
  g_systimr.interval = 0;

  /* Then process the timer expiration */

  sched_timer_expiration();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_intialize().
 *   On return, the current up-time should be available from
 *   up_timer_gettime() and the interval timer is ready for use (but not
 *   actively timing.
 *
 *   Provided by platform-specific code and called from the architecture-
 *   specific logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

void arm_timer_initialize(void)
{
  /* Disable and attach the Match 1 compare interrupt handler. */

  up_disable_irq(BCM_IRQ_TIMER1);
  (void)irq_attach(BCM_IRQ_TIMER1, bcm_systimr_interrupt, NULL);
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   arm_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

#ifndef CONFIG_CLOCK_TIMEKEEPING
int up_timer_gettime(FAR struct timespec *ts)
{
  uint64_t now;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  /* Read the 64-bit system timer */

  flags = enter_critical_section();
  now   = bcm_read_systimr();
  leave_critical_section(flags);

  /* Return the value of the timer */

  bcm_convert_systimr(now, ts);
  return OK;
}
#else
int up_timer_getcounter(FAR uint64_t *cycles)
{
  irqstate_t flags;

  DEBUGASSERT(cycles != NULL);

  flags = enter_critical_section();
  *cycles = bcm_read_systimr();

  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: up_timer_getmask
 *
 * Description:
 *   To be provided
 *
 * Input Parameters:
 *   mask - Location to return the 64-bit mask
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CLOCK_TIMEKEEPING
void up_timer_getmask(FAR uint64_t *mask)
{
  DEBUGASSERT(mask != NULL);
  *mask = 0xffffffffull;
}
#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   sched_timer_expiration() will not be called unless the timer is
 *   restarted with up_timer_start().
 *
 *   If, as a race condition, the timer has already expired when this
 *   function is called, then that pending interrupt must be cleared so
 *   that up_timer_start() and the remaining time of zero should be
 *   returned.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.  ts may be zero in which case the
 *        time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

int up_timer_cancel(FAR struct timespec *ts)
{
  uint64_t elapsed = 0;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  /* Disable the Match 1 comparison interrupt */

  flags = enter_critical_section();
  up_disable_irq(BCM_IRQ_TIMER1);

  /* Check if the timer was actually running */

  if (!g_systimr.running)
    {
      goto errout;
    }

  g_systimr.running = false;

  /* Get the time elapsed time since the interval timer was started */

  elapsed = bcm_read_systimr() - g_systimr.start;
  if (elapsed >= g_systimr.interval)
    {
      goto errout;
    }

  g_systimr.interval = 0;

  /* Return the value remaining on the timer */

  leave_critical_section(flags);
  bcm_convert_systimr(g_systimr.interval - elapsed, ts);
  return OK;

errout:
  g_systimr.running  = false;
  g_systimr.interval = 0;
  leave_critical_section(flags);

  ts->tv_sec = 0;
  ts->tv_nsec = 0;
  return -ENODATA;
}

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  sched_timer_expiration() will be
 *   called at the completion of the timeout (unless up_timer_cancel
 *   is called to stop the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until sched_timer_expiration() is
 *        called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

int up_timer_start(FAR const struct timespec *ts)
{
  irqstate_t flags;
  uint64_t interval;
  uint64_t now;

  DEBUGASSERT(ts != NULL);

  /* Convert the time to microseconds.  WARNING:  Bad things might happen if
   * this interval is very small!
   */

  interval = (uint64_t)ts->tv_sec * USEC_PER_SEC +
             (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Make certain that the Match 0 comparison interrupt is disabled */

  flags = enter_critical_section();
  up_disable_irq(BCM_IRQ_TIMER1);

  /* Configure the Match 1 comparison reigster*/

  now = bcm_read_systimr();
  putreg32(now + interval, BCM_SYSTIMR_C1);

  g_systimr.start    = now;
  g_systimr.interval = interval;
  g_systimr.running  = true;

  /* Enable the comparison interrupt */

  up_enable_irq(BCM_IRQ_TIMER1);
  leave_critical_section(flags);
  return OK;
}
