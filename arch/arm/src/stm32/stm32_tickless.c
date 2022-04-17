/****************************************************************************
 * arch/arm/src/stm32/stm32_tickless.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Ansync Labs. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Konstantin Berezenko <kpberezenko@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * STM32 Timer Usage
 *
 * This implementation uses one timer:  A free running timer to provide
 * the current time and a capture/compare channel for timed-events.
 *
 * BASIC timers that are found on some STM32 chips (timers 6 and 7) are
 * incompatible with this implementation because they don't have capture/
 * compare channels.  There are two interrupts generated from our timer,
 * the overflow interrupt which drives the timing handler and the capture/
 * compare interrupt which drives the interval handler.  There are some low
 * level timer control functions implemented here because the API of
 * stm32_tim.c does not provide adequate control over capture/compare
 * interrupts.
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
#include <debug.h>

#include "arm_internal.h"
#include "stm32_tim.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only TIM2 and TIM5 timers may be 32-bits in width
 *
 * Reference Table 2 of en.DM00042534.pdf
 */

#undef HAVE_32BIT_TICKLESS

#if (CONFIG_STM32_TICKLESS_TIMER == 2 && \
      !defined(STM32_STM32F10XX) && \
      !defined(STM32_STM32L15XX)) \
 || (CONFIG_STM32_TICKLESS_TIMER == 5 && \
      !defined(STM32_STM32F10XX))
 #define HAVE_32BIT_TICKLESS 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_tickless_s
{
  uint8_t timer;               /* The timer/counter in use */
  uint8_t channel;             /* The timer channel to use for intervals */
  struct stm32_tim_dev_s *tch; /* Handle returned by stm32_tim_init() */
  uint32_t frequency;
  uint32_t overflow;           /* Timer counter overflow */
  volatile bool pending;       /* True: pending task */
  uint32_t period;             /* Interval period */
  uint32_t base;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t stm32_getreg16(uint8_t offset)
{
  return getreg16(g_tickless.base + offset);
}

/****************************************************************************
 * Name: stm32_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_putreg16(uint8_t offset, uint16_t value)
{
  putreg16(value, g_tickless.base + offset);
}

/****************************************************************************
 * Name: stm32_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_modifyreg16(uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(g_tickless.base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32_tickless_enableint
 ****************************************************************************/

static inline void stm32_tickless_enableint(int channel)
{
  stm32_modifyreg16(STM32_BTIM_DIER_OFFSET, 0, 1 << channel);
}

/****************************************************************************
 * Name: stm32_tickless_disableint
 ****************************************************************************/

static inline void stm32_tickless_disableint(int channel)
{
  stm32_modifyreg16(STM32_BTIM_DIER_OFFSET, 1 << channel, 0);
}

/****************************************************************************
 * Name: stm32_tickless_ackint
 ****************************************************************************/

static inline void stm32_tickless_ackint(int channel)
{
  stm32_putreg16(STM32_BTIM_SR_OFFSET, ~(1 << channel));
}

/****************************************************************************
 * Name: stm32_tickless_getint
 ****************************************************************************/

static inline uint16_t stm32_tickless_getint(void)
{
  return stm32_getreg16(STM32_BTIM_SR_OFFSET);
}

/****************************************************************************
 * Name: stm32_tickless_setchannel
 ****************************************************************************/

static int stm32_tickless_setchannel(uint8_t channel)
{
  uint16_t ccmr_orig   = 0;
  uint16_t ccmr_val    = 0;
  uint16_t ccmr_mask   = 0xff;
  uint16_t ccer_val    = stm32_getreg16(STM32_GTIM_CCER_OFFSET);
  uint8_t  ccmr_offset = STM32_GTIM_CCMR1_OFFSET;

  /* Further we use range as 0..3; if channel=0 it will also overflow here */

  if (--channel > 4)
    {
      return -EINVAL;
    }

  /* Assume that channel is disabled and polarity is active high */

  ccer_val &= ~((GTIM_CCER_CC1P | GTIM_CCER_CC1E) <<
                GTIM_CCER_CCXBASE(channel));

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

#if STM32_NBTIM > 0
  if (g_tickless.base == STM32_TIM6_BASE
#endif
#if STM32_NBTIM > 1
      || g_tickless.base == STM32_TIM7_BASE
#endif
#if STM32_NBTIM > 0
     )
    {
      return -EINVAL;
    }
#endif

  /* Frozen mode because we don't want to change the GPIO, preload register
   * disabled.
   */

  ccmr_val = (ATIM_CCMR_MODE_FRZN << ATIM_CCMR1_OC1M_SHIFT);

  /* Set polarity */

  ccer_val |= ATIM_CCER_CC1P << GTIM_CCER_CCXBASE(channel);

  /* Define its position (shift) and get register offset */

  if ((channel & 1) != 0)
    {
      ccmr_val  <<= 8;
      ccmr_mask <<= 8;
    }

  if (channel > 1)
    {
      ccmr_offset = STM32_GTIM_CCMR2_OFFSET;
    }

  ccmr_orig  = stm32_getreg16(ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  stm32_putreg16(ccmr_offset, ccmr_orig);
  stm32_putreg16(STM32_GTIM_CCER_OFFSET, ccer_val);

  return OK;
}

/****************************************************************************
 * Name: stm32_interval_handler
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
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

static void stm32_interval_handler(void)
{
  tmrinfo("Expired...\n");

  /* Disable the compare interrupt now. */

  stm32_tickless_disableint(g_tickless.channel);
  stm32_tickless_ackint(g_tickless.channel);

  g_tickless.pending = false;

  nxsched_timer_expiration();
}

/****************************************************************************
 * Name: stm32_timing_handler
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

static void stm32_timing_handler(void)
{
  g_tickless.overflow++;

  STM32_TIM_ACKINT(g_tickless.tch, GTIM_SR_UIF);
}

/****************************************************************************
 * Name: stm32_tickless_handler
 *
 * Description:
 *   Generic interrupt handler for this timer.  It checks the source of the
 *   interrupt and fires the appropriate handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int stm32_tickless_handler(int irq, void *context, void *arg)
{
  int interrupt_flags = stm32_tickless_getint();

  if (interrupt_flags & ATIM_SR_UIF)
    {
      stm32_timing_handler();
    }

  if (interrupt_flags & (1 << g_tickless.channel))
    {
      stm32_interval_handler();
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
  switch (CONFIG_STM32_TICKLESS_TIMER)
    {
#ifdef CONFIG_STM32_TIM1
      case 1:
        g_tickless.base = STM32_TIM1_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM2
      case 2:
        g_tickless.base = STM32_TIM2_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM3
      case 3:
        g_tickless.base = STM32_TIM3_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM4
      case 4:
        g_tickless.base = STM32_TIM4_BASE;
        break;
#endif
#ifdef CONFIG_STM32_TIM5
      case 5:
        g_tickless.base = STM32_TIM5_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM6
      case 6:

        /* Basic timers not supported by this implementation */

        DEBUGASSERT(0);
        break;
#endif

#ifdef CONFIG_STM32_TIM7
      case 7:

        /* Basic timers not supported by this implementation */

        DEBUGASSERT(0);
        break;
#endif

#ifdef CONFIG_STM32_TIM8
      case 8:
        g_tickless.base = STM32_TIM8_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM9
      case 9:
        g_tickless.base = STM32_TIM9_BASE;
        break;
#endif
#ifdef CONFIG_STM32_TIM10
      case 10:
        g_tickless.base = STM32_TIM10_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM11
      case 11:
        g_tickless.base = STM32_TIM11_BASE;
        break;
#endif
#ifdef CONFIG_STM32_TIM12
      case 12:
        g_tickless.base = STM32_TIM12_BASE;
        break;
#endif
#ifdef CONFIG_STM32_TIM13
      case 13:
        g_tickless.base = STM32_TIM13_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM14
      case 14:
        g_tickless.base = STM32_TIM14_BASE;
        break;
#endif
#ifdef CONFIG_STM32_TIM15
      case 15:
        g_tickless.base = STM32_TIM15_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM16
      case 16:
        g_tickless.base = STM32_TIM16_BASE;
        break;
#endif

#ifdef CONFIG_STM32_TIM17
      case 17:
        g_tickless.base = STM32_TIM17_BASE;
        break;
#endif

      default:
        DEBUGASSERT(0);
    }

  /* Get the TC frequency that corresponds to the requested resolution */

  g_tickless.frequency = USEC_PER_SEC / (uint32_t)CONFIG_USEC_PER_TICK;
  g_tickless.timer     = CONFIG_STM32_TICKLESS_TIMER;
  g_tickless.channel   = CONFIG_STM32_TICKLESS_CHANNEL;
  g_tickless.pending   = false;
  g_tickless.period    = 0;
  g_tickless.overflow  = 0;

  tmrinfo("timer=%d channel=%d frequency=%lu Hz\n",
           g_tickless.timer, g_tickless.channel, g_tickless.frequency);

  g_tickless.tch = stm32_tim_init(g_tickless.timer);
  if (!g_tickless.tch)
    {
      tmrerr("ERROR: Failed to allocate TIM%d\n", g_tickless.timer);
      DEBUGASSERT(0);
    }

  STM32_TIM_SETCLOCK(g_tickless.tch, g_tickless.frequency);

  /* Set up to receive the callback when the counter overflow occurs */

  STM32_TIM_SETISR(g_tickless.tch, stm32_tickless_handler, NULL, 0);

  /* Initialize interval to zero */

  STM32_TIM_SETCOMPARE(g_tickless.tch, g_tickless.channel, 0);

  /* Setup compare channel for the interval timing */

  stm32_tickless_setchannel(g_tickless.channel);

  /* Set timer period */

#ifdef HAVE_32BIT_TICKLESS
  STM32_TIM_SETPERIOD(g_tickless.tch, UINT32_MAX);
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  g_oneshot_maxticks = UINT32_MAX;
#endif
#else
  STM32_TIM_SETPERIOD(g_tickless.tch, UINT16_MAX);
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  g_oneshot_maxticks = UINT16_MAX;
#endif
#endif

  /* Initialize the counter */

  STM32_TIM_SETMODE(g_tickless.tch, STM32_TIM_MODE_UP);

  /* Start the timer */

  STM32_TIM_ACKINT(g_tickless.tch, ~0);
  STM32_TIM_ENABLEINT(g_tickless.tch, GTIM_DIER_UIE);
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

  DEBUGASSERT(g_tickless.tch && ts);

  /* Temporarily disable the overflow counter.  NOTE that we have to be
   * careful here because  stm32_tc_getpending() will reset the pending
   * interrupt status.  If we do not handle the overflow here then, it will
   * be lost.
   */

  flags    = enter_critical_section();

  overflow = g_tickless.overflow;
  counter  = STM32_TIM_GETCOUNTER(g_tickless.tch);
  pending  = STM32_TIM_CHECKINT(g_tickless.tch, GTIM_SR_UIF);
  verify   = STM32_TIM_GETCOUNTER(g_tickless.tch);

  /* If an interrupt was pending before we re-enabled interrupts,
   * then the overflow needs to be incremented.
   */

  if (pending)
    {
      STM32_TIM_ACKINT(g_tickless.tch, GTIM_SR_UIF);

      /* Increment the overflow count and use the value of the
       * guaranteed to be AFTER the overflow occurred.
       */

      overflow++;
      counter = verify;

      /* Update tickless overflow counter. */

      g_tickless.overflow = overflow;
    }

  leave_critical_section(flags);

  tmrinfo("counter=%lu (%lu) overflow=%lu, pending=%i\n",
         (unsigned long)counter,  (unsigned long)verify,
         (unsigned long)overflow, pending);
  tmrinfo("frequency=%lu\n", g_tickless.frequency);

  /* Convert the whole thing to units of microseconds.
   *
   *   frequency = ticks / second
   *   seconds   = ticks * frequency
   *   usecs     = (ticks * USEC_PER_SEC) / frequency;
   */
#ifdef HAVE_32BIT_TICKLESS
  usec = ((((uint64_t)overflow << 32) + (uint64_t)counter) * USEC_PER_SEC) /
         g_tickless.frequency;
#else
  usec = ((((uint64_t)overflow << 16) + (uint64_t)counter) * USEC_PER_SEC) /
         g_tickless.frequency;
#endif

  /* And return the value of the timer */

  sec         = (uint32_t)(usec / USEC_PER_SEC);
  ts->tv_sec  = sec;
  ts->tv_nsec = (usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  tmrinfo("usec=%llu ts=(%lu, %lu)\n",
          usec, (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  return OK;
}

#ifdef CONFIG_CLOCK_TIMEKEEPING

/****************************************************************************
 * Name: up_timer_getcounter
 *
 * Description:
 *   To be provided
 *
 * Input Parameters:
 *   cycles - 64-bit return value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int up_timer_getcounter(uint64_t *cycles)
{
  *cycles = (uint64_t)STM32_TIM_GETCOUNTER(g_tickless.tch);
  return OK;
}

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

void up_timer_getmask(uint64_t *mask)
{
  DEBUGASSERT(mask != NULL);
#ifdef HAVE_32BIT_TICKLESS
  *mask = UINT32_MAX;
#else
  *mask = UINT16_MAX;
#endif
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_timer_expiration() will not be called unless the timer is
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

int up_timer_cancel(struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint64_t sec;
  uint64_t nsec;
  uint32_t count;
  uint32_t period;

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
          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }

      leave_critical_section(flags);
      return OK;
    }

  /* Yes.. Get the timer counter and period registers and disable the compare
   * interrupt.
   */

  tmrinfo("Cancelling...\n");

  /* Disable the interrupt. */

  stm32_tickless_disableint(g_tickless.channel);

  count  = STM32_TIM_GETCOUNTER(g_tickless.tch);
  period = g_tickless.period;

  g_tickless.pending = false;
  leave_critical_section(flags);

  /* Did the caller provide us with a location to return the time
   * remaining?
   */

  if (ts != NULL)
    {
      /* Yes.. then calculate and return the time remaining on the
       * oneshot timer.
       */

      tmrinfo("period=%lu count=%lu\n",
             (unsigned long)period, (unsigned long)count);

#ifndef HAVE_32BIT_TICKLESS
      if (count > period)
        {
          /* Handle rollover */

          period += UINT16_MAX;
        }
      else if (count == period)
#else
      if (count >= period)
#endif
        {
          /* No time remaining */

          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
          return OK;
        }

      /* The total time remaining is the difference.  Convert that
       * to units of microseconds.
       *
       *   frequency = ticks / second
       *   seconds   = ticks * frequency
       *   usecs     = (ticks * USEC_PER_SEC) / frequency;
       */

      usec        = (((uint64_t)(period - count)) * USEC_PER_SEC) /
                    g_tickless.frequency;

      /* Return the time remaining in the correct form */

      sec         = usec / USEC_PER_SEC;
      nsec        = ((usec) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (unsigned long)nsec;

      tmrinfo("remaining (%lu, %lu)\n",
             (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  nxsched_timer_expiration() will be
 *   called at the completion of the timeout (unless up_timer_cancel
 *   is called to stop the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until nxsched_timer_expiration() is
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

int up_timer_start(const struct timespec *ts)
{
  uint64_t usec;
  uint64_t period;
  uint32_t count;
  irqstate_t flags;

  tmrinfo("ts=(%lu, %lu)\n",
          (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
  DEBUGASSERT(ts);
  DEBUGASSERT(g_tickless.tch);

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

  /* Get the timer counter frequency and determine the number of counts need
   * to achieve the requested delay.
   *
   *   frequency = ticks / second
   *   ticks     = seconds * frequency
   *             = (usecs * frequency) / USEC_PER_SEC;
   */

  period = (usec * (uint64_t)g_tickless.frequency) / USEC_PER_SEC;
  count  = STM32_TIM_GETCOUNTER(g_tickless.tch);

  tmrinfo("usec=%llu period=%08llx\n", usec, period);

  /* Set interval compare value. Rollover is fine,
   * channel will trigger on the next period.
   */
#ifdef HAVE_32BIT_TICKLESS
  DEBUGASSERT(period <= UINT32_MAX);
  g_tickless.period = (uint32_t)(period + count);
#else
  DEBUGASSERT(period <= UINT16_MAX);
  g_tickless.period = (uint16_t)(period + count);
#endif

  STM32_TIM_SETCOMPARE(g_tickless.tch, g_tickless.channel,
                       g_tickless.period);

  /* Enable interrupts.  We should get the callback when the interrupt
   * occurs.
   */

  stm32_tickless_ackint(g_tickless.channel);
  stm32_tickless_enableint(g_tickless.channel);

  g_tickless.pending = true;
  leave_critical_section(flags);
  return OK;
}
#endif /* CONFIG_SCHED_TICKLESS */
