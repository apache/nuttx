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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <debug.h>

#include "arm_internal.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_gpt.h"
#include "imxrt_irq.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only alarm option is currently supported */

#ifndef CONFIG_SCHED_TICKLESS_ALARM
#  error Interval timer support is not supported yet, please select alarm
#endif

/* The Peripheral Clock (ipg_clk) is selected as the GPT clock source.
 *
 * REVISIT: Here we assume that the Peripheral Clock is 16.6 MHz. That is:
 *
 * PRECLK_CLOCK_ROOT = IPG_CLOCK_ROOT / IMXRT_PERCLK_PODF_DIVIDER
 * where IPG_CLOCK_ROOT = 150 MHz and IMXRT_PERCLK_PODF_DIVIDER = 9
 *
 * Those clocks are set in imxrt_clockconfig.c, but makros are defined in
 * board level section (file board.h) so clock settings may actually vary
 * when using different boards.
 *
 * So, Peripheral Clock Frequency = 16.6 MHz
 */

#define GPT_CLOCK 16600000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_tickless_s
{
  uint8_t timer;                   /* The timer/counter in use */
  uint8_t out_compare;             /* Number of output compare channel */
  uint32_t frequency;              /* Frequency of the timer */
  uint32_t overflow;               /* Timer counter overflow */
  uint32_t irq;                    /* Interrupt number */
  volatile bool pending;           /* True: pending task */
  uint32_t base;                   /* Base address of the timer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_get_counter
 *
 * Description:
 *   Get counter value and add it to overflow value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Counter value
 *
 ****************************************************************************/

static uint64_t imxrt_get_counter(void)
{
  return getreg32(g_tickless.base + IMXRT_GPT_CNT_OFFSET) | \
                     ((uint64_t)g_tickless.overflow << 32);
}

/****************************************************************************
 * Name: imxrt_interval_handler
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

static void imxrt_interval_handler(void)
{
  struct timespec tv;
  uint32_t regval;

  /* Disable the compare interrupt for now */

  regval = getreg32(g_tickless.base + IMXRT_GPT_IR_OFFSET);
  regval &= ~(1 << (g_tickless.out_compare - 1));
  putreg32(regval, g_tickless.base + IMXRT_GPT_IR_OFFSET);

  /* And clear it */

  putreg32((1 << (g_tickless.out_compare - 1)),
           g_tickless.base + IMXRT_GPT_SR_OFFSET);

  g_tickless.pending = false;

  up_timer_gettime(&tv);
  nxsched_alarm_expiration(&tv);
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

static void imxrt_timing_handler(void)
{
  g_tickless.overflow++;

  /* Clear interrupt bit */

  putreg32(GPT_SR_ROV, g_tickless.base + IMXRT_GPT_SR_OFFSET);
}

/****************************************************************************
 * Name: imxrt_tickless_handler
 *
 * Description:
 *   Generic interrupt handler for this timer.  It checks the source of the
 *   interrupt and fires the appropriate handler.
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int imxrt_tickless_handler(int irq, void *context, void *arg)
{
  uint32_t interrupt_flags;
  interrupt_flags = getreg32(g_tickless.base + IMXRT_GPT_SR_OFFSET);

  /* The free-run timer has reached its maximum value */

  if (interrupt_flags & GPT_SR_ROV)
    {
      imxrt_timing_handler();
    }

  /* Compare interrupt was generated */

  if (interrupt_flags & (1 << (g_tickless.out_compare - 1)))
    {
      imxrt_interval_handler();
    }

  return OK;
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
  uint32_t regval;
  int prescaler;

  switch (CONFIG_IMXRT_TICKLESS_TIMER)
    {
      case 1:
        g_tickless.base = IMXRT_GPT1_BASE;
        g_tickless.irq = IMXRT_IRQ_GPT1;
        imxrt_clockall_gpt_bus();
        imxrt_clockall_gpt_serial();
        break;
      case 2:
        g_tickless.base = IMXRT_GPT2_BASE;
        g_tickless.irq = IMXRT_IRQ_GPT2;
        imxrt_clockall_gpt2_bus();
        imxrt_clockall_gpt2_serial();
        break;
      default:
        tmrerr("ERROR: Timer number invalid or not configured: %d\n",
                CONFIG_IMXRT_TICKLESS_TIMER);
        break;
    }

  /* Get the TC frequency that corresponds to the requested resolution */

  up_disable_irq(g_tickless.irq);

  g_tickless.frequency    = USEC_PER_SEC / (uint32_t)CONFIG_USEC_PER_TICK;
  g_tickless.timer        = CONFIG_IMXRT_TICKLESS_TIMER;
  g_tickless.out_compare  = CONFIG_IMXRT_TICKLESS_CHANNEL;
  g_tickless.pending      = false;
  g_tickless.overflow     = 0;

  tmrinfo("timer=%d channel=%d frequency=%lu Hz\n",
           g_tickless.timer, g_tickless.out_compare, g_tickless.frequency);

  /* Set clock source of the timer and enable free-run mode */

  regval = getreg32(g_tickless.base + IMXRT_GPT_CR_OFFSET);
  regval |= GPT_CR_CLKSRC_IPG | GPT_CR_FRR;
  putreg32(regval, g_tickless.base + IMXRT_GPT_CR_OFFSET);

  /* Set the prescaler register */

  prescaler = GPT_CLOCK / g_tickless.frequency;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0xfff)
    {
      prescaler = 0xfff;
    }

  /* Set the prescaler value */

  putreg32(prescaler, g_tickless.base + IMXRT_GPT_PR_OFFSET);

  /* Atache the interrupt handler */

  if (irq_attach(g_tickless.irq, imxrt_tickless_handler, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      tmrerr("ERROR: Failed to attach GPT timer IRQ\n");
    }

  up_enable_irq(g_tickless.irq);

  /* Initialize interval to zero */

  putreg32(0, g_tickless.base + IMXRT_GPT_OCR1_OFFSET + \
          (4 * (g_tickless.out_compare - 1)));

  /* Initialize the counter and enable interrupts */

  regval = getreg32(g_tickless.base + IMXRT_GPT_IR_OFFSET);
  regval |= GPT_IR_ROVIE | (1 << (g_tickless.out_compare -1));
  putreg32(regval, g_tickless.base + IMXRT_GPT_IR_OFFSET);

  regval = getreg32(g_tickless.base + IMXRT_GPT_CR_OFFSET);
  regval |= GPT_CR_ENMOD;
  putreg32(regval, g_tickless.base + IMXRT_GPT_CR_OFFSET);

  /* Eneable the timer */

  regval = getreg32(g_tickless.base + IMXRT_GPT_CR_OFFSET);
  regval |= GPT_CR_EN;
  putreg32(regval, g_tickless.base + IMXRT_GPT_CR_OFFSET);
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
  uint32_t verify;
  uint32_t overflow;
  uint32_t sec;
  int pending;
  irqstate_t flags;

  /* Temporarily disable the overflow counter */

  flags    = enter_critical_section();

  overflow = g_tickless.overflow;
  counter  = getreg32(g_tickless.base + IMXRT_GPT_CNT_OFFSET);
  pending  = getreg32(g_tickless.base + IMXRT_GPT_SR_OFFSET) & GPT_SR_ROV;
  verify   = getreg32(g_tickless.base + IMXRT_GPT_CNT_OFFSET);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then the overflow needs to be incremented.
   */

  if (pending)
    {
      /* Clear the rollover interrupt */

      putreg32(GPT_SR_ROV, g_tickless.base + IMXRT_GPT_SR_OFFSET);

      /* Increment the overflow count and use the value of the
       * guaranteed to be AFTER the overflow occurred.
       */

      overflow++;
      counter = verify;

      /* Update tickless overflow counter. */

      g_tickless.overflow = overflow;
    }

  leave_critical_section(flags);

  /* Convert the whole thing to units of microseconds.
   *
   *   frequency = ticks / second
   *   seconds   = ticks * frequency
   *   usecs     = (ticks * USEC_PER_SEC) / frequency;
   */

  usec = ((((uint64_t)overflow << 32) + (uint64_t)counter) * \
          USEC_PER_SEC) / g_tickless.frequency;

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

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

int up_alarm_start(const struct timespec *ts)
{
  size_t offset = 1;
  uint64_t tm = ((uint64_t)ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec) /
                NSEC_PER_TICK;
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  /* Set compare value for output compare channel */

  putreg32(tm, g_tickless.base + IMXRT_GPT_OCR1_OFFSET + \
          (4 * (g_tickless.out_compare - 1)));

  /* Clear interrupt bits */

  putreg32((1 << (g_tickless.out_compare - 1)) | GPT_SR_ROV,
            g_tickless.base + IMXRT_GPT_SR_OFFSET);

  /* Enable interrupts */

  regval = getreg32(g_tickless.base + IMXRT_GPT_IR_OFFSET);
  regval |= GPT_IR_ROVIE | (1 << (g_tickless.out_compare - 1));
  putreg32(regval, g_tickless.base + IMXRT_GPT_IR_OFFSET);

  g_tickless.pending = true;

  /* If we have already passed this time, there is a chance we didn't set the
   * compare register in time and we've missed the interrupt. If we don't
   * catch this case, we won't interrupt until a full loop of the clock.
   *
   * Since we can't make assumptions about the clock speed and tick rate,
   * we simply keep adding an offset to the current time, until we can leave
   * certain that the interrupt is going to fire as soon as we leave the
   * critical section.
   */

  while (tm <= imxrt_get_counter())
    {
      tm = imxrt_get_counter() + offset++;
      putreg32(tm, g_tickless.base + IMXRT_GPT_OCR1_OFFSET + \
              (4 * (g_tickless.out_compare - 1)));
    }

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

int up_alarm_cancel(struct timespec *ts)
{
  uint64_t nsecs =  (((uint64_t)g_tickless.overflow << 32) | \
                    getreg32(g_tickless.base + IMXRT_GPT_CNT_OFFSET)) * \
                    NSEC_PER_TICK;
  uint32_t regval;

  ts->tv_sec = nsecs / NSEC_PER_SEC;
  ts->tv_nsec = nsecs - ts->tv_sec * NSEC_PER_SEC;

  /* Disable the compare interrupt */

  regval = getreg32(g_tickless.base + IMXRT_GPT_IR_OFFSET);
  regval &= ~(1 << (g_tickless.out_compare - 1));
  putreg32(regval, g_tickless.base + IMXRT_GPT_IR_OFFSET);

  return 0;
}

#endif /* CONFIG_SCHED_TICKLESS */
