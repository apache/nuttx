/****************************************************************************
 * arch/arm/src/imxrt/imxrt_tickless.c
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
 * Only alarm option selected by CONFIG_SCHED_TICKLESS_ALARM is currently
 * suported for iMXRT.
 *
 ****************************************************************************/

/****************************************************************************
 * iMXRT Timer Usage
 *
 * This implementation uses one timer:  A free running timer to provide
 * the current time and a capture/compare channel for timed-events.
 *
 * This timer can be either General Purpose Timer (GPT) 1 or 2, which can
 * be set by CONFIG_IMXRT_TICKLESS_TIMER. CONFIG_IMXRT_TICKLESS_CHANNEL
 * selects which channel generates the interrupt for compare value.
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
#include "hardware/xmc4_scu.h"
#include "hardware/xmc4_ccu4.h"

// #ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only alarm option is currently supported */

#ifdef CONFIG_SCHED_TICKLESS_ALARM
#error Alarm support is not supported by xmc
#endif

#define GPT_CLOCK 60000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_tickless_s
{
  uint32_t overflow;     /* Timer counter overflow */
  volatile bool pending; /* True: pending task */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_interval_handler
 *
 * Description:
 *   Called when the timer counter matches the compare register
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
  g_tickless.pending = false;
  nxsched_timer_expiration();
}

/****************************************************************************
 * Name: imxrt_timing_handler
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
    g_tickless.overflow++;
}

/****************************************************************************
 * Public Functions
 ***************************************************************************/

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
  g_tickless.pending = false;
  g_tickless.overflow = 0;

  /* Enable CCU clock */
  putreg32(SCU_CLK_CCUC, XMC4_SCU_CLKSET);

  /*****************************/
  /* Initialize Interval Timer */
  /*****************************/

  // Apply reset
  putreg32(SCU_PR0_CCU41RS, XMC4_SCU_PRSET0);
  putreg32(SCU_PR0_CCU41RS, XMC4_SCU_PRCLR0);

  /* Enable CC40 and CC41 Slices */
  putreg32(CCU4_GIDLC_CS0I_MASK, XMC4_CCU41_GIDLC);
  putreg32(CCU4_GIDLC_CS1I_MASK, XMC4_CCU41_GIDLC);

  /* Enable the prescaler and set value*/
  putreg32(CCU4_GIDLC_SPRB_MASK, XMC4_CCU41_GIDLC);
  putreg32(0, XMC4_CCU41_CC40PSC);
  putreg32(0, XMC4_CCU41_CC41PSC);

  /* Enable Single shot mode*/
  putreg32(CCU4_CC4_TC_TSSM_MASK, XMC4_CCU41_CC41TC);

  /* Concate CC40 and CC41 Slices together*/
  putreg32(CCU4_CC4_CMC_TCE_MASK, XMC4_CCU41_CC41CMC);

  /* Attach the interrupt handler */
  irq_attach(XMC4_IRQ_CCU41_SR0, (xcpt_t)xmc4_interval_handler, NULL);

  /* Enable Interrupt */
  up_enable_irq(XMC4_IRQ_CCU41_SR0);

  /***************************/
  /* Initialize Timing Timer */
  /***************************/

  // Apply reset
  putreg32(SCU_PR0_CCU40RS, XMC4_SCU_PRSET0);
  putreg32(SCU_PR0_CCU40RS, XMC4_SCU_PRCLR0);

  /* Enable CC40 and CC41 Slices */
  putreg32(CCU4_GIDLC_CS0I_MASK, XMC4_CCU40_GIDLC);
  putreg32(CCU4_GIDLC_CS1I_MASK, XMC4_CCU40_GIDLC);

  /* Enable the prescaler and set value*/
  putreg32(CCU4_GIDLC_SPRB_MASK, XMC4_CCU40_GIDLC);
  putreg32(0, XMC4_CCU40_CC40PSC);
  putreg32(0, XMC4_CCU40_CC41PSC);

  /* Concate CC40 and CC41 Slices together*/
  putreg32(CCU4_CC4_CMC_TCE_MASK, XMC4_CCU40_CC41CMC);

  /* Set Period of the timer to max */
  putreg32(UINT16_MAX, XMC4_CCU40_CC40PRS);
  putreg32(UINT16_MAX, XMC4_CCU40_CC41PRS);

  /* Enable Period Match Interrupt */
  putreg32(CCU4_CC4_INTE_PME_MASK, XMC4_CCU40_CC41INTE);
  up_enable_irq(XMC4_IRQ_CCU40_SR0);

  /* Attach the interrupt handler */
  irq_attach(XMC4_IRQ_CCU40_SR0, (xcpt_t)xmc4_timing_handler, NULL);

  /* Enable shadow transfer */
  putreg32(CCU4_GCSS_S0SE_MASK, XMC4_CCU40_GCSS);
  putreg32(CCU4_GCSS_S1SE_MASK, XMC4_CCU40_GCSS);

  /* Start the timing timer */
  putreg32(CCU4_CC4_TCSET_TRBS_MASK, XMC4_CCU40_CC41TCSET);
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
  counter = (getreg32(XMC4_CCU40_CC41TIMER) << 16) & 0xFFFF00000 + getreg32(XMC4_CCU40_CC40TIMER);
  overflow = g_tickless.overflow;

  leave_critical_section(flags);

  usec = (uint64_t)(overflow * UINT32_MAX + counter) * USEC_PER_SEC / GPT_CLOCK;
  sec = (uint32_t)(usec / USEC_PER_SEC);
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
  irqstate_t flags;

  tmrinfo("ts=(%lu, %lu)\n",
          (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(ts);

  /* Was an interval already running? */
  flags = enter_critical_section();
  if (g_tickless.pending)
  {
    tmrinfo("Already running... cancelling\n");
    up_timer_cancel(NULL);
  }

  uint64_t period = (float)((uint64_t)ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec) * GPT_CLOCK / NSEC_PER_SEC;
  putreg32(period & 0xFFFF, XMC4_CCU41_CC40PRS);
  putreg32((period >> 16) & 0xFFFF, XMC4_CCU41_CC41PRS);

  /* Enable interrupt */
  putreg32(CCU4_CC4_INTE_PME_MASK, XMC4_CCU41_CC41INTE);

  /* Enable shadow transfer */
  putreg32(CCU4_GCSS_S1SE_MASK, XMC4_CCU41_GCSS);
  putreg32(CCU4_GCSS_S0SE_MASK, XMC4_CCU41_GCSS);

  /* Start timer*/
  putreg32(CCU4_CC4_TCSET_TRBS_MASK, XMC4_CCU41_CC40TCSET);
  putreg32(CCU4_CC4_TCSET_TRBS_MASK, XMC4_CCU41_CC41TCSET);

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
  irqstate_t flags;

  /* Was the timer running? */

  flags = enter_critical_section();
  if (!g_tickless.pending)
  {
    /* No.. Just return zero timer remaining and successful cancellation.
     * This function may execute at a high rate with no timer running
     * (as when pre-emption is enabled and disabled).
     */

    if (ts)
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

  tmrinfo("Cancelling...\n");

  /* Disable interrupt */

  uint32_t regval;
  regval = getreg32(XMC4_CCU41_CC41INTE);
  regval &= ~CCU4_CC4_INTE_PME_MASK;
  putreg32(regval, XMC4_CCU41_CC41INTE);
  g_tickless.pending = false;
  leave_critical_section(flags);

  if (ts != NULL)
  {
    uint32_t count = (getreg32(XMC4_CCU41_CC41TIMER) << 16) & 0xFFFF00000 + getreg32(XMC4_CCU41_CC40TIMER);
    uint64_t nsecs = count * (NSEC_PER_SEC / GPT_CLOCK);

    ts->tv_sec = nsecs / NSEC_PER_SEC;
    ts->tv_nsec = nsecs - ts->tv_sec * NSEC_PER_SEC;

    tmrinfo("remaining (%lu, %lu)\n",
            (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  }

  return 0;
}

// #endif /* CONFIG_SCHED_TICKLESS */
