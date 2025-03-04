/****************************************************************************
 * arch/arm/src/xmc4/xmc4_tickless.c
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
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void up_timer_initialize(void): Initializes the timer facilities.
 *     Called early in the initialization sequence (by up_initialize()).
 *   int up_timer_gettime(struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(const struct timespec *ts): Start (or re-starts)
 *     the interval timer.
 *
 * The RTOS will provide the following interfaces for use by the platform-
 * specific interval timer implementation:
 *
 *   void nxsched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 *
 * NOTE
 * Alarm option is NOT supported by XMC and never will.
 * Hardware restrictions.
 *
 ****************************************************************************/

/****************************************************************************
 * XMC Timer Usage
 *
 * This implementation uses two timers:
 *  - One freerun timer to get the time since startup
 *  - One oneshoe timer to wait the desired delay
 *
 * For now, user cannot choose the CCU used. This implementation uses the
 * CCU40 for timing, and CCU41 for interval. Contributions are welcome
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
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <debug.h>

#include "arm_internal.h"
#include <arch/board/board.h>
#include "hardware/xmc4_scu.h"
#include "hardware/xmc4_ccu4.h"
#include "xmc4_ccu4.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only alarm option can be supporter by xmc.
 * Indeed, xmc CCU compare value cannot be updated
 * on the flight. It's updated via shadow registers and
 * these are loaded to compare value register only on overflow
 */

#ifdef CONFIG_SCHED_TICKLESS_ALARM
#  error Alarm support is not supported by xmc
#endif

/* The XMC only have 16 bits timers, so whatever the resolution this max is
 * reached very quickly. Therefore we force the user to enable this max.
 */

#ifndef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
#  error XMC tickless feature need to have a max delay for sleep
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xmc4_tickless_s
{
  uint32_t overflow;     /* Timer counter overflow */
  uint32_t frequency;    /* Frequency Timers */
  volatile bool pending; /* True: pending task */
  uint32_t period;       /* Interval period */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct xmc4_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_interval_handler
 *
 * Description:
 *   Called when the oneshot timer reaches its period
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void xmc4_interval_handler(void *arg)
{
  tmrinfo("Expired...\n");

  /* Stop the timer */

  putreg32(CCU4_CC4_TCCLR_TRBC_MASK, XMC4_CCU41_CC40TCCLR);

  g_tickless.pending = false;
  nxsched_timer_expiration();
}

/****************************************************************************
 * Name: xmc_timing_handler
 *
 * Description:
 *   Timer interrupt callback.  When the freerun timer counter overflows,
 *   this interrupt will occur.  We will just increment an overflow count.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void xmc4_timing_handler(void)
{
  tmrinfo("Overflow");
  g_tickless.overflow++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_initialize().
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

void up_timer_initialize(void)
{
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  uint64_t max_delay;
#endif
  int ret;

  g_tickless.pending = false;
  g_tickless.overflow = 0;
  g_tickless.frequency = USEC_PER_SEC / (uint32_t)CONFIG_USEC_PER_TICK;

  /* Enable CCU clock */

  putreg32(SCU_CLK_CCUC, XMC4_SCU_CLKSET);

  /* Enable CCU clock during sleep */

  putreg32(SCU_SLEEPCR_CCUCR | SCU_SLEEPCR_SYSSEL, XMC4_SCU_SLEEPCR);

  uint32_t divisor;
  uint32_t pssiv;
  ret = xmc4_ccu4_divisor(USEC_PER_SEC / (uint32_t)CONFIG_USEC_PER_TICK,
                          &divisor,
                          &pssiv);
  g_tickless.frequency = BOARD_CCU_FREQUENCY / divisor;
  if (ret < 0)
    {
      tmrerr("ERROR: xmc4_ccu4_divisor failed: %d\n", ret);
      return ret;
    }

  tmrinfo("frequency=%lu, divisor=%lu, cmr=%08lx\n",
          (unsigned long)g_tickless.frequency, (unsigned long)divisor,
          (unsigned long)pssiv);

  /* Initialize Interval Timer
   *
   * Ths timer is configured to be a oneshot timer, that has
   * a resolution that matches the USEC_PER_TICK, and
   * will be started in up_timer_start and uses its period
   * (not compare value) to trigger an interrupt.
   */

  /* Apply reset */

  putreg32(SCU_PR0_CCU41RS, XMC4_SCU_PRSET0);
  putreg32(SCU_PR0_CCU41RS, XMC4_SCU_PRCLR0);

  /* Enable CC40 Slice */

  putreg32(CCU4_GIDLC_CS0I_MASK, XMC4_CCU41_GIDLC);

  /* Enable the prescaler and set value */

  putreg32(CCU4_GIDLC_SPRB_MASK, XMC4_CCU41_GIDLC);
  putreg32(pssiv, XMC4_CCU41_CC40PSC);

#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  max_delay = (float)UINT16_MAX * USEC_PER_SEC / g_tickless.frequency;
  g_oneshot_maxticks = max_delay / CONFIG_USEC_PER_TICK;
#endif

  /* Enable Single shot mode */

  putreg32(CCU4_CC4_TC_TSSM_MASK, XMC4_CCU41_CC40TC);

  /* Attach the interrupt handler */

  irq_attach(XMC4_IRQ_CCU41_SR0, (xcpt_t)xmc4_interval_handler, NULL);

  /* Enable Interrupt */

  up_enable_irq(XMC4_IRQ_CCU41_SR0);

  /* Initialize Timing Timer
   *
   * This timer is configure to be a freerun timer that
   * has a resolution that matches the USEC_PER_TICK.
   */

  /* Apply reset */

  putreg32(SCU_PR0_CCU40RS, XMC4_SCU_PRSET0);
  putreg32(SCU_PR0_CCU40RS, XMC4_SCU_PRCLR0);

  /* Enable CC40 */

  putreg32(CCU4_GIDLC_CS0I_MASK, XMC4_CCU40_GIDLC);

  /* Enable the prescaler and set value */

  putreg32(CCU4_GIDLC_SPRB_MASK, XMC4_CCU40_GIDLC);
  putreg32(pssiv, XMC4_CCU40_CC40PSC);

  /* Set Period of the timer to max */

  putreg32(UINT16_MAX, XMC4_CCU40_CC40PRS);

  /* Enable Period Match Interrupt */

  putreg32(CCU4_CC4_INTE_PME_MASK, XMC4_CCU40_CC40INTE);
  up_enable_irq(XMC4_IRQ_CCU40_SR0);

  /* Attach the interrupt handler */

  irq_attach(XMC4_IRQ_CCU40_SR0, (xcpt_t)xmc4_timing_handler, NULL);

  /* Enable shadow transfer */

  putreg32(CCU4_GCSS_S0SE_MASK, XMC4_CCU40_GCSS);

  /* Start the timing timer */

  putreg32(CCU4_CC4_TCSET_TRBS_MASK, XMC4_CCU40_CC40TCSET);
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   up_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, struct timespec *ts);
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

int up_timer_gettime(struct timespec *ts)
{
  uint64_t usec;
  uint32_t counter;
  uint32_t overflow;
  uint32_t sec;
  irqstate_t flags;

  /* Temporarily disable the overflow counter */

  flags = enter_critical_section();

  counter = getreg32(XMC4_CCU40_CC40TIMER);
  overflow = g_tickless.overflow;

  leave_critical_section(flags);

  usec = (uint64_t)(overflow * UINT16_MAX + counter)
          * USEC_PER_SEC / g_tickless.frequency;
  sec = usec / USEC_PER_SEC;
  ts->tv_sec = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("usec=%llu ts=(%lu, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  nxsched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur.  When
 *        the alarm occurs the timer logic will call
 *        nxsched_alarm_expiration().
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

int up_timer_start(const struct timespec *ts)
{
  uint64_t usec;
  uint64_t period;
  irqstate_t flags;

  tmrinfo("ts=(%lu, %lu)\n",
          (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(ts);

  /* Was an interval already running? */

  flags = enter_critical_section();
  if (g_tickless.pending)
    {
      /* Yes.. then cancel it */

      tmrinfo("Already running... cancelling\n");
      up_timer_cancel(NULL);
    }

  /* Express the delay in microseconds */

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  /* Compute periods of the timers to match delay to wait */

  period = usec * (uint64_t)g_tickless.frequency / USEC_PER_SEC;
  putreg32(period, XMC4_CCU41_CC40PRS);

  tmrinfo("usec=%llu period=%08llx\n", usec, period);

  DEBUGASSERT(period <= UINT16_MAX);
  g_tickless.period = period;

  /* Enable interrupt */

  putreg32(CCU4_CC4_INTE_PME_MASK, XMC4_CCU41_CC40INTE);

  /* Enable shadow transfer */

  putreg32(CCU4_GCSS_S0SE_MASK, XMC4_CCU41_GCSS);

  /* Start timer */

  putreg32(CCU4_CC4_TCSET_TRBS_MASK, XMC4_CCU41_CC40TCSET);

  g_tickless.pending = true;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_alarm_expiration() will not be called unless the alarm is
 *   restarted with up_alarm_start().
 *
 *   If, as a race condition, the alarm has already expired when this
 *   function is called, then time returned is the current time.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the expiration time.  The current time should
 *        returned if the alarm is not active.  ts may be NULL in which
 *        case the time is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_alarm_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

int up_timer_cancel(struct timespec *ts)
{
  uint32_t regval;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  irqstate_t flags;
  uint32_t period;
  uint32_t count;

  /* Was the timer running? */

  flags = enter_critical_section();
  if (!g_tickless.pending)
    {
      /* No.. Just return zero timer remaining and successful cancellation.
       * This function may execute at a high rate with no timer running
       * (as when pre-emption is enabled and disabled).
       */

      if (ts != NULL)
        {
          ts->tv_sec = 0;
          ts->tv_nsec = 0;
        }

      leave_critical_section(flags);
      return OK;
    }

  /* Yes.. Get the timer counter and period registers and disable the compare
   * interrupt.
   */

  regval = getreg32(XMC4_CCU41_CC40INTE);
  regval &= ~CCU4_CC4_INTE_PME_MASK;
  putreg32(regval, XMC4_CCU41_CC40INTE);
  putreg32(CCU4_CC4_TCCLR_TCC_MASK | CCU4_CC4_TCCLR_TRBC_MASK,
           XMC4_CCU41_CC40TCCLR);

  count = getreg32(XMC4_CCU41_CC40TIMER);
  period = g_tickless.period;
  g_tickless.pending = false;
  leave_critical_section(flags);

  tmrinfo("Cancelling...\n");

  if (ts != NULL)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      tmrinfo("period=%lu count=%lu\n",
             (unsigned long)period, (unsigned long)count);

      /* The total time remaining is the difference.  Convert that
       * to units of microseconds.
       *
       *   frequency = ticks / second
       *   seconds   = ticks * frequency
       *   usecs     = (ticks * USEC_PER_SEC) / frequency;
       */

      usec = (((uint64_t)(period - count)) * USEC_PER_SEC) /
             g_tickless.frequency;

      sec         = usec / USEC_PER_SEC;
      nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;
      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (unsigned long)nsec;

      tmrinfo("remaining count : %lu (%lu, %lu)\n", count,
              (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

#endif /* CONFIG_SCHED_TICKLESS */
