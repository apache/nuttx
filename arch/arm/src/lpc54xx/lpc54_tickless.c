/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_tickless.c
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

#include <errno.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/lpc54_rit.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef min
#  define min(a,b) (a < b ? a : b)
#endif

#define COUNTER_MAX 0x0000ffffffffffffllu

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_to_reset = COUNTER_MAX / 2;
static uint64_t g_to_reset_next = COUNTER_MAX / 2 + COUNTER_MAX / 4;
static uint64_t g_to_end = COUNTER_MAX / 2 + COUNTER_MAX / 4 + COUNTER_MAX / 8; /* any alarm should no last more than COUNTER_MAX/8 */
static struct timespec g_max_ts;

static uint64_t g_common_div;
static uint64_t g_min_ticks;
static uint64_t g_min_nsec;

static uint64_t g_reset_ticks = 1000;  /* Ticks to add to force a reset */

static struct timespec g_base_ts;      /* Time base */
static uint64_t g_base_rest;           /* Rest of ticks that is < g_min_ticks */

static struct timespec g_alarm_ts;     /* alarm_time to set on next interrupt, used if not already g_armed */

static bool g_alarm_time_set = false;  /* true if alarm_time set and need to be processed */
static bool g_call = false;            /* true if callback should be called on next interrupt */
static bool g_forced_int = false;      /* true if interrupt was forced with mask, no reset */
static bool g_armed = false;           /* true if alarm is g_armed for next match */
static uint32_t g_synch = 0;           /* Synch all calls, recursion is possible */
static irqstate_t g_flags;

static uint32_t g_cached_ctrl;
static uint64_t g_cached_mask;
static uint64_t g_cached_compare;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Some timer HW functions */

static inline void lpc54_set_counter(uint64_t value)
{
  putreg32(0, LPC54_RIT_COUNTER);
  putreg16((uint32_t)(value >> 32), LPC54_RIT_COUNTERH);
  putreg32((uint32_t)(value & 0xffffffffllu), LPC54_RIT_COUNTER);
}

static uint64_t lpc54_get_counter(void)
{
  uint32_t ls;
  uint16_t ms;
  uint16_t verify;

  do
    {
      ms     = getreg16(LPC54_RIT_COUNTERH);
      ls     = getreg32(LPC54_RIT_COUNTER);
      verify = getreg16(LPC54_RIT_COUNTERH);
    }
  while (verify != ms);

  return (uint64_t)ms << 32 | (uint64_t)ls;
}

static void lpc54_set_compare(uint64_t value)
{
  irqstate_t flags;

  if (value != g_cached_compare)
    {
      g_cached_compare = value;

      flags = enter_critical_section();
      putreg32(0, LPC54_RIT_COMPVAL);
      putreg16((uint32_t)(value >> 32), LPC54_RIT_COMPVALH);
      putreg32((uint32_t)(value & 0xffffffffllu), LPC54_RIT_COMPVAL);
      leave_critical_section();
    }
}

static inline uint64_t lpc54_get_compare(void)
{
  return g_cached_compare;
}

static void lpc54_set_mask(uint64_t value)
{
  irqstate_t flags;

  if (value != g_cached_mask)
    {
      g_cached_mask = value;

      flags = enter_critical_section();
      putreg32(0, LPC54_RIT_MASK);
      putreg16((uint32_t)(value >> 32), LPC54_RIT_MASKH);
      putreg32((uint32_t)(value & 0xffffffffllu), LPC54_RIT_MASK);
      leave_critical_section();
      putreg32(value,
              );
    }
}

static inline uint64_t lpc54_get_mask(void)
{
  return g_cached_mask;
}

static inline bool lpc54_get_ctrl_bit(uint32_t bit)
{
  return (g_cached_ctrl & bit) != 0;
}

static inline void lpc54_set_ctrl_bit(uint32_t bit, bool value)
{
  if (lpc54_get_ctrl_bit(bit) != value)
    {
      if (value)
        {
          g_cached_ctrl |= bit;
        }
      else
        {
          g_cached_ctrl &= ~bit;
        }

      putreg32(g_cached_ctrl, LPC54_RIT_CTRL);
    }
}

static inline void lpc54_set_reset_on_match(bool value)
{
  lpc54_set_ctrl_bit(RIT_CTRL_ENCLR, value);
}

static inline bool lpc54_get_reset_on_match(void)
{
  return lpc54_get_ctrl_bit(RIT_CTRL_ENCLR);
}

static inline void lpc54_set_enable(bool value)
{
  lpc54_set_ctrl_bit(RIT_CTRL_EN, value);
}

static inline bool lpc54_get_enable(void)
{
  return lpc54_get_ctrl_bit(RIT_CTRL_EN);
}

static inline void lpc54_clear_interrupt(void)
{
  putreg32(g_cached_ctrl | RIT_CTRL_INT, LPC54_RIT_CTRL);
}

static inline bool lpc54_get_interrupt(void)
{
  return (getreg32(LPC54_RIT_CTRL) & RIT_CTRL_INT) != 0;
}

/* Converters */

static uint32_t common_div(uint32_t a, uint32_t b)
{
  while (b != 0)
    {
      int h = a % b;
      a = b;
      b = h;
    }

  return a;
}

static void lpc54_ts_add(const struct timespec *ts1,
                         const struct timespec *ts2,
                         struct timespec *ts3)
{
  time_t sec = ts1->tv_sec + ts2->tv_sec;
  long nsec  = ts1->tv_nsec + ts2->tv_nsec;

  if (nsec >= NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec++;
    }

  ts3->tv_sec  = sec;
  ts3->tv_nsec = nsec;
}

static void lpc54_ts_sub(const struct timespec *ts1,
                         const struct timespec *ts2,
                         struct timespec *ts3)
{
  time_t sec;
  long nsec;

  if (ts1->tv_sec < ts2->tv_sec)
    {
      sec  = 0;
      nsec = 0;
    }
  else if (ts1->tv_sec == ts2->tv_sec && ts1->tv_nsec <= ts2->tv_nsec)
    {
      sec  = 0;
      nsec = 0;
    }
  else
    {
      sec = ts1->tv_sec - ts2->tv_sec;
      if (ts1->tv_nsec < ts2->tv_nsec)
        {
          nsec = (ts1->tv_nsec + NSEC_PER_SEC) - ts2->tv_nsec;
          sec--;
        }
      else
        {
          nsec = ts1->tv_nsec - ts2->tv_nsec;
        }
    }

  ts3->tv_sec = sec;
  ts3->tv_nsec = nsec;
}

static inline uint64_t lpc54_ts2tick(const struct timespec *ts)
{
  return ((uint64_t)ts->tv_sec * LPC54_CCLK +
          ((uint64_t)ts->tv_nsec / g_min_nsec * g_min_ticks));
}

static uint64_t lpc54_tick2ts(uint64_t ticks, struct timespec *ts,
                              bool with_rest)
{
  uint64_t ticks_whole;
  uint64_t ticks_rest = 0;

  if (with_rest)
    {
      uint64_t ticks_mult = ticks / g_min_ticks;
      ticks_whole = ticks_mult * g_min_ticks;
      ticks_rest = ticks - ticks_whole;
    }
  else
    {
      ticks_whole = ticks;
    }

  ts->tv_sec = ticks_whole / LPC54_CCLK;
  ts->tv_nsec = ((ticks_whole % LPC54_CCLK) / g_min_ticks) * g_min_nsec;

  return ticks_rest;
}

/* Logic functions */

static inline void lpc54_sync_up(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  if (g_synch == 0)
    {
      g_flags = flags;
    }

  g_synch++;
}

static inline void lpc54_sync_down(void)
{
  g_synch--;
  if (g_synch == 0)
    {
      leave_critical_section(g_flags);
    }
}

/* Assuming safe timer state, force interrupt, no reset possible */

static inline void lpc54_force_int(void)
{
  g_forced_int = true;
  lpc54_set_reset_on_match(false);
  lpc54_set_mask(COUNTER_MAX);
  lpc54_set_compare(COUNTER_MAX);
}

/* Init all vars, g_forced_int should not be cleared */

static inline void lpc54_init_timer_vars(void)
{
  g_alarm_time_set = false;
  g_call           = false;
  g_armed          = false;
}

/* Calc g_reset_ticks and set compare to g_to_reset */

static void lpc54_calibrate_init(void)
{
  uint64_t counter       = lpc54_get_counter();
  uint64_t counter_after = lpc54_get_counter();

  counter_after = g_to_reset + counter;
  counter_after = counter_after - counter;

  /* Shift to to Reset */

  lpc54_set_compare(counter_after);

  counter_after = lpc54_get_counter();
  g_reset_ticks   = (counter_after - counter) * 2;
}

/* Process current and set timer in default safe state */

static void lpc54_save_timer(bool from_isr)
{
  if (g_forced_int) /* special case of forced interrupt by mask */
    {
      g_forced_int = false;
      lpc54_set_compare(COUNTER_MAX);
      lpc54_set_mask(0);
      lpc54_clear_interrupt();
    }
  else
    {
      /* Process reset if any */

      uint64_t match = lpc54_get_compare();

      /* Move to end, no resets during processing */

      lpc54_set_compare(COUNTER_MAX);
      lpc54_set_mask(0);

      if (from_isr || lpc54_get_interrupt())
        {
          if (lpc54_get_reset_on_match()) /* Was reset? */
            {
              struct timespec match_ts;
              g_base_rest = lpc54_tick2ts(match + g_base_rest,
                                        &match_ts, true);
              lpc54_ts_add(&g_base_ts, &match_ts, &g_base_ts);
            }

          lpc54_clear_interrupt();
        }
    }
}

/* Assuming safe timer state, true if set, false - time is in the past */

static bool lpc54_set_safe_compare(uint64_t compare_to_set)
{
  uint64_t counter;
  bool reset;
  bool reset_after;

  if (compare_to_set < g_to_reset)
    {
      lpc54_set_reset_on_match(false);
    }
  else
    {
      lpc54_set_reset_on_match(true);
    }

  lpc54_set_compare(compare_to_set);

  /* Check if ok */

  reset       = lpc54_get_interrupt();
  counter     = lpc54_get_counter();
  reset_after = lpc54_get_interrupt();

  if (reset != reset_after)
    {
      /* Was a reset get new counter */

      counter = lpc54_get_counter();
    }

  if (reset_after || (!reset_after && compare_to_set > counter))
    {
      return true;
    }
  else
    {
      lpc54_set_compare(COUNTER_MAX);

      return false;
    }
}

/* Assuming safe timer state, set_safe_compare in loop */

static void lpc54_looped_forced_set_compare(void)
{
  uint32_t i = 1;

  bool result =
    lpc54_set_safe_compare(lpc54_get_counter() + g_reset_ticks);

  while (!result)
    {
      i++;
      result =
        lpc54_set_safe_compare(lpc54_get_counter() + g_reset_ticks * i);
    }
}

/* Assuming safe timer state, true if set, false - time is in the past */

static bool lpc54_set_calc_arm(uint64_t curr, uint64_t to_set, bool arm)
{
  uint64_t calc_time;
  bool set;

  if (curr < g_to_reset_next)
    {
      calc_time = min(g_to_reset_next, to_set);
    }
  else
    {
      if (curr < g_to_end)
        {
          calc_time = min(curr + g_reset_ticks, to_set);
        }
      else
        {
          lpc54_looped_forced_set_compare();
          return true;
        }
    }

  set = lpc54_set_safe_compare(calc_time);
  if (arm && set && (calc_time == to_set))
    {
      g_armed = true;
    }

  return set;
}

/* Assuming safe timer state, try to set compare for normal operation */

static void lpc54_set_default_compare(uint64_t curr)
{
  bool result = lpc54_set_calc_arm(curr, COUNTER_MAX, false);
  if (!result)
    {
      result = lpc54_set_calc_arm(lpc54_get_counter(), COUNTER_MAX,
                                     false);
      if (!result)
        {
          lpc54_looped_forced_set_compare();
        }
    }
}

/* Calculates ticks to set from g_alarm_ts and g_base_ts/g_base_rest,
 * COUNTER_MAX if overflow.
 */

static inline uint64_t lpc54_calc_to_set(void)
{
  struct timespec diff_ts;
  struct timespec ovf_ts;

  lpc54_ts_sub(&g_alarm_ts, &g_base_ts, &diff_ts);

  lpc54_ts_sub(&diff_ts, &g_max_ts, &ovf_ts);
  if (ovf_ts.tv_sec == 0 && ovf_ts.tv_nsec == 0) /* check overflow */
    {
      return (lpc54_ts2tick(&diff_ts) - g_base_rest);
    }
  else
    {
      return COUNTER_MAX;
    }
}

/* Assuming safe timer state, used by isr: sets default compare,
 * calls alarm.
 */

static inline void lpc54_tl_alarm(uint64_t curr)
{
  lpc54_init_timer_vars();
  lpc54_set_default_compare(curr);

#ifdef CONFIG_SCHED_TICKLESS_ALARM
  struct timespec ts;
  up_timer_gettime(&ts);
  nxsched_alarm_expiration(&ts);
#else
  nxsched_timer_expiration();
#endif
}

/* Interrupt handler */

static int lpc54_tl_isr(int irq, void *context, void *arg)
{
  uint64_t curr;

  lpc54_sync_up();
  lpc54_save_timer(true);

  curr = lpc54_get_counter();
  if (g_call)
    {
      lpc54_tl_alarm(curr);
    }
  else
    {
      if (g_armed)
        {
          lpc54_tl_alarm(curr); /* g_armed - g_call alarm */
        }
      else
        {
          if (g_alarm_time_set) /* need to set alarm time */
            {
              uint64_t toset = lpc54_calc_to_set();

              if (toset > curr)
                {
                  if (toset > g_to_end)
                    {
                      lpc54_set_default_compare(curr);
                    }
                  else
                    {
                      bool set = lpc54_set_calc_arm(curr, toset, true);
                      if (!set)
                        {
                          lpc54_tl_alarm(curr);
                        }
                    }
                }
              else
                {
                  lpc54_tl_alarm(curr);
                }
            }
          else
            {
              lpc54_set_default_compare(curr);
            }
        }
    }

  lpc54_sync_down();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  g_cached_ctrl     = getreg32(LPC54_RIT_CTRL);
  g_cached_ctrl    &= ~RIT_CTRL_INT; /* Set interrupt to 0 */
  g_cached_mask     = getreg32(LPC54_RIT_MASK);
  g_cached_compare  = getreg32(LPC54_RIT_COMPVAL);

  g_common_div      = common_div(NSEC_PER_SEC, LPC54_CCLK);
  g_min_ticks       = LPC54_CCLK / g_common_div;
  g_min_nsec        = NSEC_PER_SEC / g_common_div;

  g_base_ts.tv_sec  = 0;
  g_base_ts.tv_nsec = 0;
  g_base_rest       = 0;

  lpc54_tick2ts(g_to_end, &g_max_ts, false);

  lpc54_set_enable(false);

  lpc54_set_compare(COUNTER_MAX);
  lpc54_set_counter(0);
  lpc54_set_mask(0);

  lpc54_set_reset_on_match(false);
  lpc54_clear_interrupt();

  irq_attach(LPC54M4_IRQ_RITIMER, lpc54_tl_isr, NULL);
  up_enable_irq(LPC54M4_IRQ_RITIMER);

  lpc54_init_timer_vars();
  lpc54_set_enable(true);
  lpc54_calibrate_init();
  leave_critical_section(flags);
}

/* No reg changes, only processing */

int up_timer_gettime(struct timespec *ts)
{
  struct timespec count_ts;
  uint64_t count;
  bool reset;

  lpc54_sync_up();

  /* Order of calls is important, reset can come during processing */

  reset = lpc54_get_interrupt();
  count = lpc54_get_counter();

  /* Not processed reset can exist */

  if (lpc54_get_reset_on_match())
    {
      bool reset_after = lpc54_get_interrupt();

      /* Was a reset during processing? get new counter */

      if (reset != reset_after)
        {
          count = lpc54_get_counter();
        }

      if (reset_after)
        {
          /* Count should be smaller then
           * COUNTER_MAX-g_to_end -> no overflow
           */

          count += lpc54_get_compare();
        }
    }

  lpc54_tick2ts(count + g_base_rest, &count_ts, false);
  lpc54_ts_add(&g_base_ts, &count_ts, ts);
  lpc54_sync_down();
  return OK;
}

int up_alarm_cancel(struct timespec *ts)
{
  lpc54_sync_up();

  /* No reg changes, only variables logic */

  if (ts != NULL)
    {
      up_timer_gettime(ts);
    }

  /* Let default setup will be done in interrupt handler or up_alarm_start */

  lpc54_init_timer_vars();

  lpc54_sync_down();
  return OK;
}

int up_alarm_start(const struct timespec *ts)
{
  uint64_t toset;
  uint64_t curr;

  lpc54_sync_up();
  lpc54_save_timer(false);
  lpc54_init_timer_vars();

  g_alarm_time_set        = true;
  g_alarm_ts.tv_sec  = ts->tv_sec;
  g_alarm_ts.tv_nsec = ts->tv_nsec;

  toset = lpc54_calc_to_set();
  curr  = lpc54_get_counter();

  if (toset > curr)
    {
      if (toset > g_to_end) /* Future set */
        {
          lpc54_set_default_compare(curr);
        }
      else
        {
          bool set = lpc54_set_calc_arm(curr, toset, true);
          if (!set) /* Signal g_call, force interrupt handler */
            {
              g_call = true;
              lpc54_force_int();
            }
        }
    }
  else /* Signal g_call, force interrupt handler */
    {
      g_call = true;
      lpc54_force_int();
    }

  lpc54_sync_down();

  return OK;
}

#ifndef CONFIG_SCHED_TICKLESS_ALARM
int up_timer_cancel(struct timespec *ts)
{
  lpc54_sync_up();

  if (ts != NULL)
    {
      struct timespec abs_ts;
      up_timer_gettime(&abs_ts);
      lpc54_ts_sub(&g_alarm_ts, &abs_ts, ts);
    }

  lpc54_init_timer_vars();
  lpc54_sync_down();
  return OK;
}

int up_timer_start(const struct timespec *ts)
{
  lpc54_sync_up();

  struct timespec abs_ts;
  up_timer_gettime(&abs_ts);
  lpc54_ts_add(&abs_ts, ts, &abs_ts);

  up_alarm_start(&abs_ts);

  lpc54_sync_down();
  return OK;
}

#endif /* CONFIG_SCHED_TICKLESS_ALARM */
#endif /* CONFIG_SCHED_TICKLESS */
