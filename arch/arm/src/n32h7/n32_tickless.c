/****************************************************************************
 * arch/arm/src/n32h7/n32_tickless.c
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
 ****************************************************************************/

/****************************************************************************
 * N32H7 Timer Usage
 *
 * This implementation uses one timer: A free running timer to provide
 * the current time and a capture/compare channel for timed-events.
 *
 * Basic timers (BTIM) are incompatible because they don't have
 * capture/compare channels. Use ATIM or GTIMA instead.
 *
 * There are two interrupts generated from our timer:
 *   - overflow interrupt  -> update 64-bit counter
 *   - compare interrupt   -> interval expiration
 *
 * Some low-level register operations are done directly because the
 * n32_tim.c API does not expose all needed controls.
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
#include <nuttx/timers/arch_timer.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "n32_tim.h"
#include "n32_dbgmcu.h"

#ifdef CONFIG_SCHED_TICKLESS

/* Timer is always 16-bit on N32H7 for tickless (we use ATIM/GTIMA) */

/* No 32-bit tickless support on N32H7 because BTIM lacks CC channel */

#define HAVE_16BIT_TICKLESS 1

#if CONFIG_N32H7_TICKLESS_CHANNEL == 1
#  define DIER_CAPT_IE       N32_ATIM_DINTEN_CC1IEN
#  define SR_CAPT_IF         N32_ATIM_STS_CC1ITF
#elif CONFIG_N32H7_TICKLESS_CHANNEL == 2
#  define DIER_CAPT_IE       N32_ATIM_DINTEN_CC2IEN
#  define SR_CAPT_IF         N32_ATIM_STS_CC2ITF
#elif CONFIG_N32H7_TICKLESS_CHANNEL == 3
#  define DIER_CAPT_IE       N32_ATIM_DINTEN_CC3IEN
#  define SR_CAPT_IF         N32_ATIM_STS_CC3ITF
#elif CONFIG_N32H7_TICKLESS_CHANNEL == 4
#  define DIER_CAPT_IE       N32_ATIM_DINTEN_CC4IEN
#  define SR_CAPT_IF         N32_ATIM_STS_CC4ITF
#else
#  error "Invalid tickless channel"
#endif

/* Note: N32H7 ATIM STS and DINTEN are 32-bit registers */

#define N32_STS_OFFSET        N32_ATIM_STS_OFFSET
#define N32_DINTEN_OFFSET     N32_ATIM_DINTEN_OFFSET

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n32_tickless_s
{
  uint8_t timer;               /* Timer number (1-18) */
  uint8_t channel;             /* Capture/compare channel (1-4) */
  uint32_t base;               /* Timer base address */
  struct n32_tim_dev_s *tch;   /* Timer device handle */
  uint32_t frequency;          /* Timer clock frequency (AHB) */
  volatile uint32_t overflow;  /* Number of overflows (16-bit counter) */
  volatile bool pending;       /* True: interval timer is pending */
  uint32_t period;             /* Interval compare value */
#ifdef CONFIG_SCHED_TICKLESS_ALARM
  uint64_t last_alrm;          /* Last alarm time (alarm mode) */
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct n32_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_getreg16/n32_putreg16 (for 16-bit registers like CNT, PSC, ARR)
 * Name: n32_getreg32/n32_putreg32/n32_modreg32 (for 32-bit registers)
 ****************************************************************************/

static inline uint16_t n32_getreg16(uint32_t offset)
{
  return getreg16(g_tickless.base + offset);
}

static inline void n32_putreg16(uint32_t offset, uint16_t value)
{
  putreg16(value, g_tickless.base + offset);
}

static inline uint32_t n32_getreg32(uint32_t offset)
{
  return getreg32(g_tickless.base + offset);
}

static inline void n32_putreg32(uint32_t offset, uint32_t value)
{
  putreg32(value, g_tickless.base + offset);
}

static inline void n32_modreg32(uint32_t offset, uint32_t clearbits,
                                uint32_t setbits)
{
  modifyreg32(g_tickless.base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: n32_tickless_enableint/n32_tickless_disableint/n32_tickless_ackint
 ****************************************************************************/

static inline void n32_tickless_enableint(void)
{
  n32_modreg32(N32_DINTEN_OFFSET, 0, DIER_CAPT_IE);
}

static inline void n32_tickless_disableint(void)
{
  n32_modreg32(N32_DINTEN_OFFSET, DIER_CAPT_IE, 0);
}

static inline void n32_tickless_ackint(void)
{
  n32_putreg32(N32_STS_OFFSET, ~SR_CAPT_IF);
}

static inline uint32_t n32_tickless_getint(void)
{
  return n32_getreg32(N32_STS_OFFSET);
}

/****************************************************************************
 * Name: n32_tickless_setchannel
 *
 * Description:
 *   Configure the timer channel as output compare (frozen mode) for interval
 *   timing.  This is similar to n32_tim_setchannel but hard-coded for the
 *   tickless timer.
 *
 ****************************************************************************/

static int n32_tickless_setchannel(uint8_t channel)
{
  uint32_t ccmr_orig;
  uint32_t ccmr_val;
  uint32_t ccmr_mask;
  uint32_t ccmr_offset;
  uint32_t ccer_val;
  uint32_t ccer_shift;
  uint32_t ccmr_mode_shift;
  uint32_t ccen_bit;

  if (channel < 1 || channel > 4)
    {
      return -EINVAL;
    }

  ccer_val = n32_getreg16(N32_ATIM_CCEN_OFFSET);
  ccer_shift = (uint32_t)(channel - 1) * 4;
  ccen_bit = N32_ATIM_CCEN_CC1EN << ccer_shift;

  /* Determine CCMOD register and shift */

  if (channel <= 2)
    {
      ccmr_offset = N32_ATIM_CCMOD1_OFFSET;
      ccmr_mode_shift = (channel == 1) ? 0 : (N32_ATIM_CCMOD1_OC2MD_SHIFT -
                        N32_ATIM_CCMOD1_OC1MD_SHIFT);
    }
  else
    {
      ccmr_offset = N32_ATIM_CCMOD2_OFFSET;
      ccmr_mode_shift = (channel == 3) ? 0 : (N32_ATIM_CCMOD2_OC4MD_SHIFT -
                        N32_ATIM_CCMOD2_OC3MD_SHIFT);
    }

  /* Clear channel enable bit */

  ccer_val &= ~ccen_bit;
  ccer_val &= ~(N32_ATIM_CCEN_CC1P << ccer_shift);

  /* Frozen mode, no preload */

  ccmr_val = (N32_ATIM_CCMOD1_OCMODE_FRZN << ccmr_mode_shift);
  ccmr_mask = 0xff << ccmr_mode_shift;

  /* Clear polarity (active high) and enable output */

  ccer_val &= ~(N32_ATIM_CCEN_CC1P << ccer_shift);
  ccer_val |= ccen_bit;

  /* Write CCMOD */

  ccmr_orig = n32_getreg16(ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  n32_putreg16(ccmr_offset, ccmr_orig);

  /* Write CCEN */

  n32_putreg16(N32_ATIM_CCEN_OFFSET, ccer_val);

  return OK;
}

/****************************************************************************
 * Name: n32_interval_handler
 *
 * Description:
 *   Called when the timer counter matches the compare register.
 *
 ****************************************************************************/

static void n32_interval_handler(void)
{
  tmrinfo("Expired...\n");

  /* Disable the compare interrupt now. */

  n32_tickless_disableint();
  n32_tickless_ackint();

  g_tickless.pending = false;

  nxsched_process_timer();
}

/****************************************************************************
 * Name: n32_timing_handler
 *
 * Description:
 *   Timer overflow interrupt handler.  Increment the overflow counter.
 *
 ****************************************************************************/

static void n32_timing_handler(void)
{
  g_tickless.overflow++;
  n32_putreg32(N32_STS_OFFSET, ~N32_ATIM_STS_UDITF);
}

/****************************************************************************
 * Name: n32_tickless_handler
 *
 * Description:
 *   Generic interrupt handler for this timer.  Checks the source of the
 *   interrupt and fires the appropriate handler.
 *
 ****************************************************************************/

static int n32_tickless_handler(int irq, void *context, void *arg)
{
  uint32_t sr = n32_tickless_getint();

  if (sr & N32_ATIM_STS_UDITF)
    {
      n32_timing_handler();
    }

  if (sr & SR_CAPT_IF)
    {
      n32_interval_handler();
    }

  return OK;
}

/****************************************************************************
 * Name: n32_tickless_get_counter64
 *
 * Description:
 *   Read the 64-bit counter value (overflow + current counter).
 *
 ****************************************************************************/

static uint64_t n32_tickless_get_counter64(void)
{
  irqstate_t flags;
  uint32_t overflow;
  uint32_t counter;
  uint32_t verify;
  int pending;

  flags = enter_critical_section();

  overflow = g_tickless.overflow;
  counter  = n32_getreg16(N32_ATIM_CNT_OFFSET);
  pending  = (n32_tickless_getint() & N32_ATIM_STS_UDITF) ? 1 : 0;
  verify   = n32_getreg16(N32_ATIM_CNT_OFFSET);

  if (pending)
    {
      /* Acknowledge the overflow interrupt and increment */

      n32_putreg32(N32_STS_OFFSET, N32_ATIM_STS_UDITF);
      overflow++;
      counter = verify;
      g_tickless.overflow = overflow;
    }

  leave_critical_section(flags);

  return ((uint64_t)overflow << 16) | (uint64_t)counter;
}

/****************************************************************************
 * Name: n32_tickless_dbgmcu_freeze
 *
 * Description:
 *   Configure the timer to stop in debug mode.
 *
 ****************************************************************************/

static void n32_tickless_dbgmcu_freeze(uint32_t base)
{
  switch (base)
    {
#ifdef CONFIG_N32H7_ATIM3
      case N32_ATIMER3_BASE:
        modifyreg32(N32_DBG_M7APB5FZ, 0, DBG_M7APB5FZ_ATIM3_STOP);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM4
      case N32_ATIMER4_BASE:
        modifyreg32(N32_DBG_M7APB5FZ, 0, DBG_M7APB5FZ_ATIM4_STOP);
        break;
#endif
      default:
        tmrerr("ERROR: Unsupported tickless timer!\n");
        PANIC();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  int timer = CONFIG_N32H7_TICKLESS_TIMER;
  int channel = CONFIG_N32H7_TICKLESS_CHANNEL;

  tmrinfo("timer=%d channel=%d\n", timer, channel);

  /* Set base address and frequency based on selected timer */

  switch (timer)
    {
#ifdef CONFIG_N32H7_ATIM3
      case 6:
        g_tickless.base = N32_ATIMER3_BASE;
        break;
#endif
#ifdef CONFIG_N32H7_ATIM4
      case 7:
        g_tickless.base = N32_ATIMER4_BASE;
        break;
#endif
      default:
        tmrerr("ERROR: Unsupported tickless timer %d\n", timer);
        PANIC();
    }

  g_tickless.timer = timer;
  g_tickless.channel = channel;
  g_tickless.frequency = USEC_PER_SEC / (uint32_t)CONFIG_USEC_PER_TICK;
  g_tickless.overflow = 0;
  g_tickless.pending = false;
  g_tickless.period = 0;

  tmrinfo("base=0x%08lx frequency=%lu Hz\n",
          (unsigned long)g_tickless.base, g_tickless.frequency);

  /* Initialize the timer hardware */

  g_tickless.tch = n32_tim_init(timer);
  if (g_tickless.tch == NULL)
    {
      tmrerr("ERROR: Failed to initialize TIM%d\n", timer);
      PANIC();
    }

  /* Set timer clock and period (max for 16-bit) */

  N32_TIM_SETCLOCK(g_tickless.tch, g_tickless.frequency);

  /* Set up interrupt handler (both update and CC) */

  N32_TIM_SETISR(g_tickless.tch, n32_tickless_handler, NULL,
                 N32_TIM_ISR_UPDATE);
  N32_TIM_SETISR(g_tickless.tch, n32_tickless_handler, NULL,
                 N32_TIM_ISR_CC);

  /* Initialize interval to zero */

  N32_TIM_SETCOMPARE(g_tickless.tch, channel, 0);

  /* Configure compare channel for interval timing */

  n32_tickless_setchannel(channel);

  /* Set timer period */

  N32_TIM_SETPERIOD(g_tickless.tch, UINT16_MAX);

  /* Initialize the counter */

  N32_TIM_SETMODE(g_tickless.tch, N32_TIM_MODE_UP);

  /* Acknowledge any pending interrupts and enable update interrupt */

  n32_putreg32(N32_STS_OFFSET, N32_ATIM_STS_UDITF | SR_CAPT_IF);
  N32_TIM_ENABLEINT(g_tickless.tch, N32_ATIM_DINTEN_UIEN);

  /* Configure debug freeze (stop timer when core halted) */

  n32_tickless_dbgmcu_freeze(g_tickless.base);

  tmrinfo("Tickless timer initialized\n");

#if defined(CONFIG_ARMV7M_SYSTICK) && defined(CONFIG_CPULOAD_PERIOD)
  nxsched_period_extclk(systick_initialize(true, N32_M7CPU_FREQUENCY, -1));
#endif
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up.
 *
 ****************************************************************************/

int up_timer_gettime(struct timespec *ts)
{
  uint64_t ticks;
  uint64_t usec;
  uint64_t sec;

  DEBUGASSERT(ts != NULL);

  if (g_tickless.tch == NULL)
    {
      ts->tv_sec = 0;
      ts->tv_nsec = 0;
      return OK;
    }

  ticks = n32_tickless_get_counter64();

  usec = (ticks * USEC_PER_SEC) / g_tickless.frequency;

  sec = usec / USEC_PER_SEC;
  ts->tv_sec = (time_t)sec;
  ts->tv_nsec = (long)((usec - (sec * USEC_PER_SEC)) * NSEC_PER_USEC);

  tmrinfo("ticks=%llu usec=%llu ts=(%lu, %lu)\n",
          ticks, usec, (unsigned long)ts->tv_sec,
          (unsigned long)ts->tv_nsec);

  return OK;
}

#ifdef CONFIG_CLOCK_TIMEKEEPING

/****************************************************************************
 * Name: up_timer_gettick
 ****************************************************************************/

int up_timer_gettick(clock_t *ticks)
{
  if (ticks == NULL)
    {
      return -EINVAL;
    }

  *ticks = (clock_t)n32_tickless_get_counter64();
  return OK;
}

/****************************************************************************
 * Name: up_timer_getmask
 ****************************************************************************/

void up_timer_getmask(clock_t *mask)
{
  DEBUGASSERT(mask != NULL);
  *mask = 0xffff;   /* 16-bit timer */
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining.
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS_ALARM
int up_timer_cancel(struct timespec *ts)
{
  irqstate_t flags;
  uint64_t usec;
  uint32_t count;
  uint32_t period;

  flags = enter_critical_section();

  if (!g_tickless.pending)
    {
      if (ts != NULL)
        {
          ts->tv_sec = 0;
          ts->tv_nsec = 0;
        }

      leave_critical_section(flags);
      return OK;
    }

  tmrinfo("Cancelling...\n");

  /* Disable the compare interrupt */

  n32_tickless_disableint();

  count  = n32_getreg16(N32_ATIM_CNT_OFFSET);
  period = g_tickless.period;

  g_tickless.pending = false;

  leave_critical_section(flags);

  if (ts != NULL)
    {
      /* Calculate remaining time */

      if (count < period)
        {
          usec = ((uint64_t)(period - count) * USEC_PER_SEC) /
                 g_tickless.frequency;
        }
      else
        {
          usec = 0;
        }

      ts->tv_sec = (time_t)(usec / USEC_PER_SEC);
      ts->tv_nsec = (long)((usec - (uint64_t)(ts->tv_sec * USEC_PER_SEC)) *
                           NSEC_PER_USEC);

      tmrinfo("Remaining: %lu sec, %lu nsec\n",
              (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);
    }

  return OK;
}
#endif /* !CONFIG_SCHED_TICKLESS_ALARM */

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS_ALARM
int up_timer_start(const struct timespec *ts)
{
  uint64_t usec;
  uint64_t period_ticks;
  uint32_t count;
  uint32_t compare;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  tmrinfo("ts=(%lu, %lu)\n",
          (unsigned long)ts->tv_sec, (unsigned long)ts->tv_nsec);

  flags = enter_critical_section();

  if (g_tickless.pending)
    {
      up_timer_cancel(NULL);
    }

  usec = (uint64_t)ts->tv_sec * USEC_PER_SEC +
         (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  period_ticks = (usec * g_tickless.frequency) / USEC_PER_SEC;

  if (period_ticks == 0)
    {
      period_ticks = 1;
    }

  count = n32_getreg16(N32_ATIM_CNT_OFFSET);
  compare = count + (uint32_t)period_ticks;

  if (compare > 0xffff)
    {
      /* Wrap around: set compare to count - (0xffff - period) */

      compare = count - (0xffff - (uint32_t)period_ticks);
    }

  g_tickless.period = compare;
  n32_putreg16(N32_ATIM_CCDAT1_OFFSET + (g_tickless.channel - 1) * 4,
               compare);

  n32_tickless_ackint();
  n32_tickless_enableint();

  g_tickless.pending = true;

  leave_critical_section(flags);

  tmrinfo("Started: count=%lu compare=%lu\n", count, compare);

  return OK;
}
#endif /* !CONFIG_SCHED_TICKLESS_ALARM */

#ifdef CONFIG_SCHED_TICKLESS_ALARM

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm timer (absolute time).
 *
 ****************************************************************************/

int up_alarm_start(const struct timespec *ts)
{
  uint64_t target_ticks;
  uint64_t current_ticks;
  uint32_t compare;
  uint32_t current_low;
  uint64_t diff;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  target_ticks = ((uint64_t)ts->tv_sec * NSEC_PER_SEC + ts->tv_nsec) /
                 NSEC_PER_TICK;

  flags = enter_critical_section();

  if (g_tickless.pending)
    {
      n32_tickless_disableint();
      g_tickless.pending = false;
    }

  current_ticks = n32_tickless_get_counter64();
  current_low = n32_getreg16(N32_ATIM_CNT_OFFSET);
  diff = target_ticks - current_ticks;

  if (diff <= 0xffff)
    {
      compare = current_low + (uint32_t)diff;
    }
  else
    {
      compare = 0xffff;
    }

  if (compare > 0xffff)
    {
      compare -= 0x10000;
    }

  g_tickless.period = compare;
  n32_putreg16(N32_ATIM_CCDAT1_OFFSET + (g_tickless.channel - 1) * 4,
               compare);

  n32_tickless_ackint();
  n32_tickless_enableint();

  g_tickless.pending = true;
  g_tickless.last_alrm = target_ticks;

  leave_critical_section(flags);

  tmrinfo("Alarm started: current=%llu target=%llu compare=%lu\n",
          current_ticks, target_ticks, compare);

  return OK;
}

/****************************************************************************
 * Name: up_alarm_cancel
 ****************************************************************************/

int up_alarm_cancel(struct timespec *ts)
{
  irqstate_t flags;
  uint64_t nsec;

  flags = enter_critical_section();

  if (!g_tickless.pending)
    {
      if (ts != NULL)
        {
          ts->tv_sec = 0;
          ts->tv_nsec = 0;
        }

      leave_critical_section(flags);
      return OK;
    }

  n32_tickless_disableint();
  g_tickless.pending = false;

  uint64_t current_ticks = n32_tickless_get_counter64();

  leave_critical_section(flags);

  if (ts != NULL)
    {
      if (g_tickless.last_alrm > current_ticks)
        {
          uint64_t diff = g_tickless.last_alrm - current_ticks;

          nsec = (diff * NSEC_PER_SEC) / g_tickless.frequency;
        }
      else
        {
          nsec = 0;
        }

      ts->tv_sec = (time_t)(nsec / NSEC_PER_SEC);
      ts->tv_nsec = (long)(nsec % NSEC_PER_SEC);
    }

  return OK;
}

#endif /* CONFIG_SCHED_TICKLESS_ALARM */

#endif /* CONFIG_SCHED_TICKLESS */
