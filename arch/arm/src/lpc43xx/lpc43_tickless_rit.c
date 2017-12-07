/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_rit.c
 *
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
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
 *
 * only controlled resets to 0 are performed, no direct set to counter
 * working counter region is from 0 to TO_END
 * all public functions are synchronized with disabled irqs
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

#include "up_arch.h"
#include "chip.h"
#include "chip/lpc43_rit.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef min
#  define min(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t TO_RESET = UINT32_MAX / 2;
static uint32_t TO_RESET_NEXT = UINT32_MAX / 2 + UINT32_MAX / 4;
static uint32_t TO_END = UINT32_MAX / 2 + UINT32_MAX / 4 + UINT32_MAX / 8; /* any alarm should no last more than UINT32_MAX/8 */
static struct timespec MAX_TS;

static uint32_t COMMON_DEV;
static uint32_t MIN_TICKS;
static uint32_t MIN_NSEC;

static uint32_t RESET_TICKS = 1000;   /* Ticks to add to force a reset */

static struct timespec base_ts;       /* Time base */
static uint32_t base_rest;            /* Rest of ticks that is < MIN_TICKS */

static struct timespec alarm_time_ts; /* alarmTime to set on next interrupt, used if not already armed */

static bool alarm_time_set = false;   /* true if alarm_time set and need to be processed */
static bool call = false;             /* true if callback should be called on next interrupt */
static bool forced_int = false;       /* true if interrupt was forced with mask, no reset */
static bool armed = false;            /* true if alarm is armed for next match */
static uint32_t synch = 0;            /* Synch all calls, recursion is possible */
static irqstate_t g_flags;

static uint32_t ctrl_cache;
static uint32_t mask_cache;
static uint32_t compare_cache;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Some timer HW functions */

static inline void lpc43_tl_set_counter(uint32_t value)
{
  putreg32(value, LPC43_RIT_COUNTER);
}

static inline uint32_t lpc43_tl_get_counter(void)
{
  return getreg32(LPC43_RIT_COUNTER);
}

static inline void lpc43_tl_set_compare(uint32_t value)
{
  if (value != compare_cache)
    {
      compare_cache = value;
      putreg32(value, LPC43_RIT_COMPVAL);
    }
}

static inline uint32_t lpc43_tl_get_compare(void)
{
  return compare_cache;
}

 static inline void lpc43_tl_set_mask(uint32_t value)
{
  if (value != mask_cache)
    {
      mask_cache = value;
      putreg32(value, LPC43_RIT_MASK);
    }
}

static inline uint32_t lpc43_tl_get_mask(void)
{
  return mask_cache;
}

static inline bool lpc43_tl_get_ctrl_bit(uint32_t bit)
{
  return ((ctrl_cache & bit)?true:false);
}

static inline void lpc43_tl_set_ctrl_bit(uint32_t bit, bool value)
{
  if (lpc43_tl_get_ctrl_bit(bit) != value)
    {
      if (value)
        {
          ctrl_cache |= bit;
        }
      else
        {
          ctrl_cache &= ~bit;
        }

      putreg32(ctrl_cache, LPC43_RIT_CTRL);
    }
}

static inline void lpc43_tl_set_reset_on_match(bool value)
{
  lpc43_tl_set_ctrl_bit(RIT_CTRL_ENCLR, value);
}

static inline bool lpc43_tl_get_reset_on_match(void)
{
  return lpc43_tl_get_ctrl_bit(RIT_CTRL_ENCLR);
}

static inline void lpc43_tl_set_enable(bool value)
{
  lpc43_tl_set_ctrl_bit(RIT_CTRL_EN, value);
}

static inline bool lpc43_tl_get_enable(void)
{
  return lpc43_tl_get_ctrl_bit(RIT_CTRL_EN);
}

static inline void lpc43_tl_clear_interrupt(void)
{
  putreg32(ctrl_cache | RIT_CTRL_INT, LPC43_RIT_CTRL);
}

static inline bool lpc43_tl_get_interrupt(void)
{
  return ((getreg32(LPC43_RIT_CTRL) & RIT_CTRL_INT)?true:false);
}

/* Converters */

static uint32_t common_dev(uint32_t a, uint32_t b)
{
  while (b != 0)
    {
      int h = a%b;
      a = b;
      b = h;
    }

  return a;
}

static void lpc43_tl_add(FAR const struct timespec *ts1,
                         FAR const struct timespec *ts2,
                         FAR struct timespec *ts3)
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

static void lpc43_tl_sub(FAR const struct timespec *ts1,
                         FAR const struct timespec *ts2,
                         FAR struct timespec *ts3)
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

static inline uint32_t lpc43_tl_ts2tick(FAR const struct timespec *ts)
{
  return (ts->tv_sec*LPC43_CCLK + (ts->tv_nsec/MIN_NSEC*MIN_TICKS));
}

static uint32_t lpc43_tl_tick2ts(uint32_t ticks, FAR struct timespec *ts,
                                 bool with_rest)
{
  uint32_t ticks_whole;
  uint32_t ticks_rest = 0;

  if (with_rest)
    {
      uint32_t ticks_mult = ticks/MIN_TICKS;
      ticks_whole = ticks_mult*MIN_TICKS;
      ticks_rest = ticks - ticks_whole;
    }
  else
    {
      ticks_whole = ticks;
    }

  ts->tv_sec = ticks_whole/LPC43_CCLK;
  ts->tv_nsec = ((ticks_whole%LPC43_CCLK)/MIN_TICKS)*MIN_NSEC;

  return ticks_rest;
}

/* Logic functions */

static inline void lpc43_tl_sync_up(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  if (synch == 0)
    {
      g_flags = flags;
    }

  synch++;
}

static inline void lpc43_tl_sync_down(void)
{
  synch--;
  if (synch == 0)
    {
      leave_critical_section(g_flags);
    }
}

/* Assuming safe timer state, force interrupt, no reset possible */

static inline void lpc43_tl_force_int(void)
{
  forced_int = true;
  lpc43_tl_set_reset_on_match(false);
  lpc43_tl_set_mask(UINT32_MAX);
  lpc43_tl_set_compare(UINT32_MAX);
}

/* Init all vars, forced_int should not be cleared */

static inline void lpc43_tl_init_timer_vars(void)
{
  alarm_time_set = false;
  call = false;
  armed = false;
}

/* Calc RESET_TICKS and set compare to TO_RESET */

static void lpc43_tl_calibrate_init(void)
{
  uint32_t counter = lpc43_tl_get_counter();

  uint32_t counter_after = lpc43_tl_get_counter();
  counter_after = TO_RESET + counter;
  counter_after = counter_after - counter;

  /* Shift to to Reset */

  lpc43_tl_set_compare(counter_after);

  counter_after = lpc43_tl_get_counter();

  RESET_TICKS = (counter_after - counter) * 2;
}

/* Process current and set timer in default safe state */

static void lpc43_tl_save_timer(bool from_isr)
{
  if (forced_int) /* special case of forced interrupt by mask */
    {
      forced_int = false;
      lpc43_tl_set_compare(UINT32_MAX);
      lpc43_tl_set_mask(0);
      lpc43_tl_clear_interrupt();
   }
  else
   {
      /* Process reset if any */

      uint32_t match = lpc43_tl_get_compare();

      /* Move to end, no resets during processing */

      lpc43_tl_set_compare(UINT32_MAX);
      lpc43_tl_set_mask(0);

     if (from_isr || lpc43_tl_get_interrupt())
        {
          if (lpc43_tl_get_reset_on_match()) /* Was reset? */
            {
              struct timespec match_ts;
              base_rest = lpc43_tl_tick2ts(match + base_rest,
                                           &match_ts, true);
              lpc43_tl_add(&base_ts, &match_ts, &base_ts);
            }

          lpc43_tl_clear_interrupt();
        }
    }
}

/* Assuming safe timer state, true if set, false - time is in the past */

static bool lpc43_tl_set_safe_compare(uint32_t compare_to_set)
{
  if (compare_to_set < TO_RESET)
    {
      lpc43_tl_set_reset_on_match(false);
    }
  else
    {
      lpc43_tl_set_reset_on_match(true);
    }

  lpc43_tl_set_compare(compare_to_set);

  /* Check if ok */

  bool reset = lpc43_tl_get_interrupt();
  uint32_t counter = lpc43_tl_get_counter();
  bool reset_after = lpc43_tl_get_interrupt();

  if (reset != reset_after)
    {
      /* Was a reset get new counter */

      counter = lpc43_tl_get_counter();
    }

  if (reset_after || (!reset_after && compare_to_set > counter))
    {
      return true;
    }
  else
    {
      lpc43_tl_set_compare(UINT32_MAX);

      return false;
    }
}

/* Assuming safe timer state, set_safe_compare in loop */

static void lpc43_tl_looped_forced_set_compare(void)
{
  uint32_t i = 1;
  bool result = lpc43_tl_set_safe_compare(
      lpc43_tl_get_counter() + RESET_TICKS); /* like in calibrateInit */

  while (!result)
    {
      i++;
      result = lpc43_tl_set_safe_compare(
          lpc43_tl_get_counter() + RESET_TICKS * i);
    }
}

/* Assuming safe timer state, true if set, false - time is in the past */

static bool lpc43_tl_set_calc_arm(uint32_t curr, uint32_t to_set, bool arm)
{

  uint32_t calc_time;

  if (curr < TO_RESET_NEXT)
    {
      calc_time = min(TO_RESET_NEXT, to_set);
    }
  else
    {
      if (curr < TO_END)
        {
          calc_time = min(curr + RESET_TICKS, to_set);
        }
      else
        {
          lpc43_tl_looped_forced_set_compare();
          return true;
        }
    }

  bool set = lpc43_tl_set_safe_compare(calc_time);

  if (arm && set && (calc_time == to_set))
    {
      armed = true;
    }

  return set;
}

/* Assuming safe timer state, try to set compare for normal operation */

static void lpc43_tl_set_default_compare(uint32_t curr)
{
  bool result = lpc43_tl_set_calc_arm(curr, UINT32_MAX, false);
  if (!result)
    {
      result = lpc43_tl_set_calc_arm(lpc43_tl_get_counter(), UINT32_MAX,
                                     false);
      if (!result)
        {
          lpc43_tl_looped_forced_set_compare();
        }
    }
}

/* Calculates ticks to set from alarm_time_ts and base_ts/base_rest,
 * UINT32_MAX if overflow.
 */

static inline uint32_t lpc43_tl_calc_to_set(void)
{
  struct timespec diff_ts;
  struct timespec ovf_ts;

  lpc43_tl_sub(&alarm_time_ts, &base_ts, &diff_ts);

  lpc43_tl_sub(&diff_ts, &MAX_TS, &ovf_ts);
  if (ovf_ts.tv_sec == 0 && ovf_ts.tv_nsec == 0) /* check overflow */
    {
      return (lpc43_tl_ts2tick(&diff_ts) - base_rest);
    }
  else
    {
      return UINT32_MAX;
    }
}

/* Assuming safe timer state, used by isr: sets default compare,
 * calls alarm.
 */

static inline void lpc43_tl_alarm(uint32_t curr)
{
  lpc43_tl_init_timer_vars();
  lpc43_tl_set_default_compare(curr);

#ifdef CONFIG_SCHED_TICKLESS_ALARM
  struct timespec ts;
  up_timer_gettime(&ts);
  sched_alarm_expiration(&ts);
#else
  sched_timer_expiration();
#endif
}

/* Interrupt handler */

static int lpc43_tl_isr(int irq, FAR void *context, FAR void *arg)
{
  lpc43_tl_sync_up();

  lpc43_tl_save_timer(true);

  uint32_t curr = lpc43_tl_get_counter();
  if (call)
    {
      lpc43_tl_alarm(curr);
    }
  else
    {
      if (armed)
        {
          lpc43_tl_alarm(curr); /* armed - call alarm */
        }
      else
        {
          if (alarm_time_set) /* need to set alarm time */
            {
              uint32_t toset = lpc43_tl_calc_to_set();

              if (toset > curr)
                {
                  if (toset > TO_END)
                    {
                      lpc43_tl_set_default_compare(curr);
                    }
                  else
                    {
                      bool set = lpc43_tl_set_calc_arm(curr, toset, true);
                      if (!set)
                        {
                          lpc43_tl_alarm(curr);
                        }
                    }
                }
              else
                {
                  lpc43_tl_alarm(curr);
                }
            }
          else
            {
              lpc43_tl_set_default_compare(curr);
            }
        }
    }

  lpc43_tl_sync_down();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_timer_initialize(void)
{
  irqstate_t flags;
  flags = enter_critical_section();

  ctrl_cache = getreg32(LPC43_RIT_CTRL);
  ctrl_cache &= ~RIT_CTRL_INT; /* Set interrupt to 0 */
  mask_cache = getreg32(LPC43_RIT_MASK);
  compare_cache = getreg32(LPC43_RIT_COMPVAL);

  COMMON_DEV = common_dev(NSEC_PER_SEC, LPC43_CCLK);
  MIN_TICKS = LPC43_CCLK/COMMON_DEV;
  MIN_NSEC = NSEC_PER_SEC/COMMON_DEV;

  base_ts.tv_sec = 0;
  base_ts.tv_nsec = 0;
  base_rest = 0;

  lpc43_tl_tick2ts(TO_END, &MAX_TS, false);

  lpc43_tl_set_enable(false);

  lpc43_tl_set_compare(UINT32_MAX);
  lpc43_tl_set_counter(0);
  lpc43_tl_set_mask(0);

  lpc43_tl_set_reset_on_match(false);
  lpc43_tl_clear_interrupt();

  irq_attach(LPC43M4_IRQ_RITIMER, lpc43_tl_isr, NULL);
  up_enable_irq(LPC43M4_IRQ_RITIMER);

  lpc43_tl_init_timer_vars();

  lpc43_tl_set_enable(true);

  lpc43_tl_calibrate_init();

  leave_critical_section(flags);
}

/* No reg changes, only processing */

int up_timer_gettime(FAR struct timespec *ts)
{
  lpc43_tl_sync_up();

  /* Order of calls is important, reset can come during processing */

  bool reset = lpc43_tl_get_interrupt();
  uint32_t count = lpc43_tl_get_counter();

  /* Not processed reset can exist */

  if (lpc43_tl_get_reset_on_match())
    {
      bool reset_after = lpc43_tl_get_interrupt();

      /* Was a reset during processing? get new counter */

      if (reset != reset_after)
        {
          count = lpc43_tl_get_counter();
        }

      if (reset_after)
        {
          /* Count should be smaller then UINT32_MAX-TO_END -> no overflow */

          count += lpc43_tl_get_compare();
        }
    }

  struct timespec count_ts;

  lpc43_tl_tick2ts(count + base_rest, &count_ts, false);

  lpc43_tl_add(&base_ts, &count_ts, ts);

  lpc43_tl_sync_down();

  return OK;
}

int up_alarm_cancel(FAR struct timespec *ts)
{
  lpc43_tl_sync_up();

  /* No reg changes, only variables logic */

  if (ts != NULL)
    {
      up_timer_gettime(ts);
    }

  /* Let default setup will be done in interrupt handler or up_alarm_start */

  lpc43_tl_init_timer_vars();

  lpc43_tl_sync_down();
  return OK;
}

int up_alarm_start(FAR const struct timespec *ts)
{
  lpc43_tl_sync_up();

  lpc43_tl_save_timer(false);

  lpc43_tl_init_timer_vars();

  alarm_time_set = true;
  alarm_time_ts.tv_sec = ts->tv_sec;
  alarm_time_ts.tv_nsec = ts->tv_nsec;

  uint32_t toset = lpc43_tl_calc_to_set();

  uint32_t curr = lpc43_tl_get_counter();

  if (toset > curr)
    {
      if (toset > TO_END) /* Future set */
        {
          lpc43_tl_set_default_compare(curr);
        }
      else
        {
          bool set = lpc43_tl_set_calc_arm(curr, toset, true);
          if (!set) /* Signal call, force interrupt handler */
            {
              call = true;
              lpc43_tl_force_int();
            }
        }
    }
  else /* Signal call, force interrupt handler */
    {
      call = true;
      lpc43_tl_force_int();
    }

  lpc43_tl_sync_down();

  return OK;
}

#ifndef CONFIG_SCHED_TICKLESS_ALARM
int up_timer_cancel(FAR struct timespec *ts)
{
  lpc43_tl_sync_up();

  if (ts != NULL)
    {
      struct timespec abs_ts;
      up_timer_gettime(&abs_ts);
      lpc43_tl_sub(&alarm_time_ts, &abs_ts, ts);
    }

  lpc43_tl_init_timer_vars();

  lpc43_tl_sync_down();
  return OK;
}

int up_timer_start(FAR const struct timespec *ts)
{
  lpc43_tl_sync_up();

  struct timespec abs_ts;
  up_timer_gettime(&abs_ts);
  lpc43_tl_add(&abs_ts, ts, &abs_ts);

  up_alarm_start(&abs_ts);

  lpc43_tl_sync_down();
  return OK;
}

#endif /* CONFIG_SCHED_TICKLESS_ALARM */
#endif /* CONFIG_SCHED_TICKLESS */
