/****************************************************************************
 * arch/avr/src/avrdx/avrdx_timerisr_tickless_alarm.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <avr/io.h>

#include "avr_internal.h"
#include "avrdx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The system timer is clocked from 32kHz internal oscillator
 * with a period of 61us. (Using division by 2 in case crystal
 * correction is used at some point.) There are also synchronization
 * delays for setting changes. Let's say we can reliably support
 * 300us intervals (and that's not going to be very precise.)
 */

#if CONFIG_USEC_PER_TICK < 300
#  error Values of usec per system clock tick below 300 are not supported
#endif

/* Default value for RTC.CMP, splits the RTC period in half */
#define RTC_HALFPERIOD_TICKS 32767

/* Constants for DIV2 setting of the RTC prescaler:
 * - nanoseconds per RTC tick (truncated)
 * - seconds per half of the RTC period
 * - nanoseconds needed to be added to the uptime every half of RTC
 *   period to compensate for truncating nanoseconds per RTC tick
 *
 * Explanation for the _CORR constants: 61035ns isn't xactly 1/16384
 * seconds, there's a remainder of 1/(64e8). That is a 2.5us error
 * over a 1 second period, 221ms over a day. The error is exactly
 * 1/390625 second which translates to 4/390625 second over 4 second
 * period (RTC period), which is 10240ns. Add that value every time
 * the RTC wraps to make the clock error-less.
 */
#define RTC_NSEC_PER_TICK 61035L
#define RTC_SEC_PER_HALFWRAP 2
#define RTC_NSEC_PER_HALFPERIOD_CORR 5120
#define RTC_NSEC_PER_FULLPERIOD_CORR 5120

/* Constants for DIV32. Same meaning as for DIV2. Currently unused.
 * In this case, the error over full RTC period (64s) is 1ns
 * so it is only corrected over full period
 */
#define RTC_NSEC_PER_TICK_32 976562L
#define RTC_SEC_PER_HALFWRAP32 32
#define RTC_NSEC_PER_HALFPERIOD_CORR32 0
#define RTC_NSEC_PER_FULLPERIOD_CORR32 1

/* Bit flag for avrdx_increment_uptime telling it to not correct
 * time rounding error for half period interrupt
 */
#define NO_HALFPERIOD_CORRECTION 0x80
/* Bit flag for avrdx_increment_uptime (or rather avrdx_check_alarm_expired)
 * telling it to not enable interrupts if busy-waiting for CMP busy flag
 * to reset
 */
#define NO_INTERRUPT_ENABLE 0x40
/* Bit mask removing context bits from context, leaving only actual flags
 * from the RTC interrupt register
 */
#define CONTEXT_INTERRUPT_MASK 0x3f

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void avrdx_rtc_await_nobusy(uint8_t wait_flag, irqstate_t irqstate);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct _g_avrdx_system_timer_s
{
  struct timespec uptime;
  struct timespec alarm;

  uint16_t last_cnt; /* This is the last value seen in RTC.CNT I/O register */
  uint8_t active:1;  /* Set if there is an active alarm */

  /* If this is set, RTC is running with prescaler set
   * to DIV32 instead of DIV2. (Currently unused.)
   */

  uint8_t low_freq_prescaler:1;

  /* If this is set, CMP register contains value needed
   * to wake the MCU up when the alarm expires
   */

  uint8_t cmp_is_set:1;

  /* If this is set, CMP interrupt ran last and OVF interrupt
   * should run next. Vice versa if this is cleared.
   */

  uint8_t cmp_intr_ran_last:1;
} g_avrdx_system_timer_s;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  avrdx_count_uptime_increment
 *
 * Description:
 *   Receive current counter value and calculate difference from previous,
 *   known value, taking wraps into account.
 *
 * Input Parameters:
 *   Current value
 *
 * Returned Value:
 *   The difference
 *
 ****************************************************************************/

static uint16_t avrdx_count_uptime_increment(uint16_t current_cnt)
{
  uint16_t increment = 0;

  if (current_cnt > g_avrdx_system_timer_s.last_cnt)
    {
      /* no overflow */

      increment = current_cnt - g_avrdx_system_timer_s.last_cnt;
    }
  else
    {
      if (current_cnt < g_avrdx_system_timer_s.last_cnt)
        {
          /* overflow and wrap.
           * The +1 in the calculation is a way to do a 16 bit
           * subtraction with a 17 bit number
           * (65536 - x == 65535 + 1 - x
           */

          increment = (65535 - g_avrdx_system_timer_s.last_cnt) \
                      + 1 + current_cnt;
        }
    }

  return increment;
}

/****************************************************************************
 * Function:  avrdx_deactivate_alarm
 *
 * Description:
 *   Does everything that is needed when the timer fires or is cancelled
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static void avrdx_deactivate_alarm(void)
{
  g_avrdx_system_timer_s.active = 0;
  g_avrdx_system_timer_s.cmp_is_set = 0;

  /* In idle state, CMP is set to half period. Having alarm set probably
   * changed that. Fix it if able. Note that this change may mean that
   * CMP interrupt fires twice during one RTC period. Interrupt handler
   * tracks which one of the OVF/CMP interrupts triggered last and will
   * ignore double fire from CMP.
   */

  if (RTC.CMP != RTC_HALFPERIOD_TICKS)
    {
      if (!(RTC.STATUS & RTC_CMPBUSY_bm))
        {
          RTC.CMP = RTC_HALFPERIOD_TICKS;
        }

      /* Otherwise will be done at some other time */
    }
}

/****************************************************************************
 * Function:  avrdx_check_alarm_expired
 *
 * Description:
 *   Compare alarm time with current time and notify NuttX if the alarm
 *   time is up. Also enable/disable interrupts as appropriate, set compare
 *   register to trigger interrupt etc.
 *
 * Input Parameters:
 *   Execution context (interrupt flags of the RTC peripheral)
 *
 * Assumptions:
 *   Only called if g_avrdx_system_timer_s.active is set.
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static void avrdx_check_alarm_expired(uint8_t context)
{
  struct timespec tv;
  int32_t sec_diff;
  uint32_t nsec_diff;
  int16_t interval;
  uint16_t temp16;
  irqstate_t irqstate;

  struct timespec *g_uptime_ts; /* shortcut variables */
  struct timespec *g_alarm_ts;

  if (!g_avrdx_system_timer_s.active)
    {
      /* Not active, not expired. Would be more effective
       * to have callers do this but that proved error-prone
       */

      return;
    }

  g_uptime_ts = &(g_avrdx_system_timer_s.uptime);
  g_alarm_ts = &(g_avrdx_system_timer_s.alarm);

  if
    ((context & CONTEXT_INTERRUPT_MASK) &&
      (\
        (g_uptime_ts->tv_sec > g_alarm_ts->tv_sec) || \
        (\
          (g_uptime_ts->tv_sec == g_alarm_ts->tv_sec) && \
          (g_uptime_ts->tv_nsec >= g_alarm_ts->tv_nsec)
        )
      )
    )
    {
      /* Alarm expired, notify the OS. Only do it when running in interrupt
       * context. Remove non-interrupt flags from the context before
       * using it.
       *
       * This possibly incurs small error (delay between alarm expiration
       * and time read) but alarm must be deactivated first. up_timer_gettime
       * will attempt to update time and that method in turn calls this one
       * and the program will run into recursion loop
       */

      avrdx_deactivate_alarm();
      up_timer_gettime(&tv);
      nxsched_alarm_expiration(&tv);
    }
  else
    {
      /* Is the alarm about to fire? Well, first thing - did we already
       * evaluate this?
       *
       * Note that g_uptime_ts may be greater than g_alarm_ts if this method
       * was not called from interrupt context.
       */

      if (g_avrdx_system_timer_s.cmp_is_set)
        {
          return;
        }

      /* Note about data types - struct timespec is defined
       * in include/time.h, tv_sec is of type time_t which is defined
       * in include/sys/types.h as uint32_t or uint64_t based
       * on CONFIG_SYSTEM_TIME64
       *
       * tv_nsec is defined as long, signed value
       */

      /* As long as the time remaining is over 6 seconds, we keep CMP
       * at half period mark. In theory, the threshold should be 4 seconds
       * but that might leave us in a situation where CMP needs to be set
       * to CNT+1, which cannot be done. Having CMP set early poses
       * no problem.
       *
       * This long series of condition is pretty much just a signedness-safe
       * way of calculating a difference
       */

      sec_diff = g_alarm_ts->tv_sec - g_uptime_ts->tv_sec;
      if (sec_diff > 6)
        {
          /* Not about to fire */

          return;
        }

      if (sec_diff == 6)
        {
          if (g_alarm_ts->tv_nsec > g_uptime_ts->tv_nsec)
            {
              /* 6 seconds and something is more than 6 seconds,
               * still considered not about to fire
               */

              return;
            }

          /* Less than 6 seconds is considered about to fire,
           * it is less than full two RTC periods
           */

          nsec_diff = g_uptime_ts->tv_nsec - g_alarm_ts->tv_nsec;
          if (nsec_diff)
            {
              sec_diff -= 1;
              nsec_diff = 1000000000 - nsec_diff;
            }
        }
      else
        {
          /* About to fire */

          if (g_alarm_ts->tv_nsec >= g_uptime_ts->tv_nsec)
            {
              nsec_diff = g_alarm_ts->tv_nsec - g_uptime_ts->tv_nsec;
            }
          else
            {
              nsec_diff = g_uptime_ts->tv_nsec - g_alarm_ts->tv_nsec;
              sec_diff -= 1;
              nsec_diff = 1000000000 - nsec_diff;
            }
        }

      /* The alarm is about to fire. Calculate remaining time in terms
       * of RTC ticks
       */

      if (sec_diff < 0)
        {
          /* The alarm time is in the past, set shortest interval
           * we can manage
           */

          interval = 4;
        }
      else
        {
          /* RTC clocked with 16384Hz crystal does 1 second every 16384
           * ticks, that's 1 << 14
           */

          interval = sec_diff * (1 << 14);
          interval += nsec_diff / RTC_NSEC_PER_TICK;
          /* Interval can now be zero because of rounding error.
           * Additionally, it's not really feasible to expect the MCU
           * to reconfigure the RTC quickly. Enforce minimum interval
           * of 4 ticks. (Remember how we triggered error
           * for CONFIG_USEC_PER_TICK less than 300us above? This is why.)
           */

          interval = (interval >= 4) ? interval : 4;
        }

      if (RTC.STATUS & RTC_CMPBUSY_bm)
        {
          /* Not good. we need to reconfigure CMP but can't.
           * Are we going to get an overflow interrupt by any chance?
           */

          if (RTC.CNT + interval < interval)
            {
              /* Yes, the counter will overflow before the alarm fires,
               * the handler will re-run this code and set the CMP then
               */

              return;
            }
          else
            {
              /* Backup option. Is CMP currently set to a value that
               * will be reached before the timer expires? (With some time
               * in reserve.)
               *
               * Note that we may not be able to wake ourselves up on time
               * using this option. If current RTC.CMP is too close
               * to the value we need it set to, we will not be able to set
               * it to the new value in time (it has synchronization time)
               * and RTC.CNT will go past new value before it actually takes
               * effect. We could compensate for that by adding +3 margin
               * to the condition below but that would leave us with the only
               * other option, which is spinning in loop waiting for the busy
               * flag to deassert.
               */

              temp16 = RTC.CMP;
              if ((RTC.CNT < temp16) && (RTC.CNT + interval > temp16))
                {
                  /* Yes. CMP currently does not have the value we need but
                   * at least it will trigger before the alarm expires so
                   * there will be another opportunity to set it properly
                   */

                  return;
                }
            }

          /* Bad times, we will now enter a busy loop and wait for CMP
           * being writable. Depending on the execution context we can
           * at least (possibly) allow something else to run.
           *
           * This should not happen unless NuttX wants very short
           * alarm intervals.
           */

          /* irqstate_t is SREG contents before cli. Context
           * equal to zero means not coming from interrupt handler,
           * enabling interrupts permitted. (Discard non-interrupt flags)
           */

          if (context & CONTEXT_INTERRUPT_MASK)
            {
              irqstate = (irqstate_t)0;
            }
          else
            {
              irqstate = (irqstate_t)CPU_I_bm;
            }

          avrdx_rtc_await_nobusy(RTC_CMPBUSY_bm,
                                 irqstate); /* Break the line for nxstyle */

          /* avrdx_rtc_await_nobusy possibly enabled interrupts
           * and in any case, the world is two RTC ticks different now.
           * Evaluate with recursion.
           */

          avrdx_check_alarm_expired(context);
          return;
        }

      /* Set compare register */

      if (RTC.CNT + interval != 65535)
        {
          RTC.CMP = RTC.CNT + interval;
        }
      else
        {
          /* No need to set CMP to be equal to PER, overflow
           * interrupt will handle this
           */

          RTC.CMP = RTC_HALFPERIOD_TICKS;
        }

      g_avrdx_system_timer_s.cmp_is_set = 1;

      /* alarm did not fire or not in interrupt context branch end */
    }
}

/****************************************************************************
 * Function:  avrdx_increment_uptime
 *
 * Description:
 *   Increment uptime according to current value in RTC.CNT
 *   (compared to the value it held when we last saw it.)
 *
 *   Also check for active alarm and possibly notify NuttX
 *
 * Input Parameters:
 *   Execution context (interrupt flags of the RTC peripheral)
 *
 * Assumptions:
 *   Interrupts are disabled by the caller
 *
 ****************************************************************************/

static void avrdx_increment_uptime(uint8_t context)
{
  uint16_t current_cnt;
  uint16_t increment;
  uint32_t nsec_per_tick;
  uint8_t temp;

  /* shortcut variable */

  struct timespec *g_uptime_ts;

  /* Read this early so there is no jitter caused by various conditions
   * being evaluated
   */

  current_cnt = RTC.CNT;

  g_uptime_ts = &(g_avrdx_system_timer_s.uptime);

  if (context & RTC_OVF_bm)
    {
      /* Called from interrupt handler triggered by overflow.
       * Correct rounding error of the system clock
       */

      if (!g_avrdx_system_timer_s.low_freq_prescaler)
        {
          g_uptime_ts->tv_nsec += RTC_NSEC_PER_FULLPERIOD_CORR;
        }
      else
        {
          g_uptime_ts->tv_nsec += RTC_NSEC_PER_FULLPERIOD_CORR32;
        }
    }

  /* Temporary variable to make nxstyle happy. Must not use
   * inline comment either
   */

  temp = context & RTC_CMP_bm;
  if ((temp) && ((context & NO_HALFPERIOD_CORRECTION) == 0))
    {
      /* Called from interrupt handler triggered by compare match
       * Correct rounding error with a value for half-period
       */

      if (!g_avrdx_system_timer_s.low_freq_prescaler)
        {
          g_uptime_ts->tv_nsec += RTC_NSEC_PER_HALFPERIOD_CORR;
        }
      else
        {
          g_uptime_ts->tv_nsec += RTC_NSEC_PER_HALFPERIOD_CORR32;
        }
    }

  /* If current_cnt equals last_cnt and we are not in any interrupt
   * context, the time did not change, return. If we are in an interrupt
   * context, then it's possible that those values equal because full
   * RTC period elapsed. However, that proved too complex to handle
   * so instead we just use both OVF and CMP interrupts, forcing current
   * and last CNT values to not be equal.
   *
   * They can still be equal if we don't handle CMP interrupt quickly
   * enough twice in a row. Not handling that. Just return.
   *
   * Apart from saving CPU time, returning early here is also important
   * to break recursive loop where check for alarm expiration tries
   * to notify NuttX, providing current time. Obtaining current time
   * triggers call to this function, that in turns checks for timer
   * expiration, loop until things break.
   *
   * (That loop is broken in the check too but skipping this prevents
   * getting more loops like that.)
   */

  increment = avrdx_count_uptime_increment(current_cnt);
  if (!increment)
    {
      return;
    }

  /* Update last seen CNT and calculate elapsed time */

  g_avrdx_system_timer_s.last_cnt = current_cnt;
  if (g_avrdx_system_timer_s.low_freq_prescaler)
    {
      nsec_per_tick = RTC_NSEC_PER_TICK_32;
    }
  else
    {
      nsec_per_tick = RTC_NSEC_PER_TICK;
    }

  g_uptime_ts->tv_nsec += nsec_per_tick * increment;

  /* Handle possible overflow of nanosecond counter */

  while (g_uptime_ts->tv_nsec >= 1000ul * 1000 * 1000)
    {
      g_uptime_ts->tv_nsec -= 1000ul * 1000 * 1000;
      g_uptime_ts->tv_sec++;
    }

  /* Evaluate alarm */

  avrdx_check_alarm_expired(context);
}

/****************************************************************************
 * Function:  avrdx_timerisr
 *
 * Description:
 *   The timer ISR executes whenever RTC counter goes above either period
 *   or compare value. It will evaluate if the timer expired (if set)
 *   and notifx NuttX in that case
 *
 ****************************************************************************/

static int avrdx_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  uint8_t intflags;
  uint8_t temp;

  intflags = RTC.INTFLAGS;

  /* Both interrupts are handled in the same way here, uptime
   * increment function will do the rest as appropriate.
   *
   * At no point these flags should be set at the same time, if CMP
   * calculates to equal PER, it is reset to half period instead
   * and overflow handles alarm expiration.
   *
   * But it can still happen when the interrupt handler doesn't
   * run on time if CMP doesn't differ from PER that much
   *
   * Process the one which was not ran last in that case
   */

  if ((intflags & (RTC_OVF_bm | RTC_CMP_bm)) == (RTC_OVF_bm | RTC_CMP_bm))
    {
      /* Both interrupts raised at the same time, make them alternate */

      if (g_avrdx_system_timer_s.cmp_intr_ran_last)
        {
          avrdx_increment_uptime(RTC_OVF_bm);
          RTC.INTFLAGS = RTC_OVF_bm;
          g_avrdx_system_timer_s.cmp_intr_ran_last = 0;
        }
      else
        {
          avrdx_increment_uptime(RTC_CMP_bm);
          RTC.INTFLAGS = RTC_CMP_bm;
          g_avrdx_system_timer_s.cmp_intr_ran_last = 1;
        }
    }
  else
    {
      if (intflags & RTC_OVF_bm)
        {
          /* Consider this: OVF interrupt fires, CNT resets to zero.
           * CNT counts to some point between 0 and CMP, then up_alarm_start
           * is called. CMP changes below current CNT in response.
           * CMP interrupt does not fire and then OVF interrupt fires
           * again. Half interrupt time correction was skipped.
           *
           * Rectify by checking the cmp_intr_ran_last flag and run
           * both handlers
           */

          if (!g_avrdx_system_timer_s.cmp_intr_ran_last)
            {
              avrdx_increment_uptime(RTC_CMP_bm);
              g_avrdx_system_timer_s.cmp_intr_ran_last = 1;
            }

          avrdx_increment_uptime(RTC_OVF_bm);
          RTC.INTFLAGS = RTC_OVF_bm;
          g_avrdx_system_timer_s.cmp_intr_ran_last = 0;
        }
      else /* ie. not true: if (intflags & RTC_OVF_bm) */
        {
          /* Consider this: there can be two alarm set with expiration
           * time falling into the same RTC period. All of them will
           * expire based on CMP interrupt. Every time the interrupt
           * fires, avrdx_increment_uptime needs to run to handle
           * the timer expiration. It also deals with time rounding
           * error correction and that must only be done once.
           *
           * That's what the NO_HALFPERIOD_CORRECTION flag is for
           */

          RTC.INTFLAGS = RTC_CMP_bm;
          temp = RTC_CMP_bm;
          if (g_avrdx_system_timer_s.cmp_intr_ran_last)
            {
              temp |= NO_HALFPERIOD_CORRECTION;
            }

          avrdx_increment_uptime(temp);
          g_avrdx_system_timer_s.cmp_intr_ran_last = 1;
        }
    }

  /* Timer expiration wants to reset CMP to its half-period value
   * but does not do it, if the CMP register is busy. Do it here
   * in that case (regrettably, it will be tested on each pass
   * but that might be better than spinning in loop for 1/8192
   * (2 tick durations) seconds with interrupts disabled.
   *
   * Can still only do it if CMP register is not busy
   */

  if ((!(g_avrdx_system_timer_s.active)) && \
      (!(RTC.STATUS & RTC_CMPBUSY_bm)) && \
      (RTC.CMP != RTC_HALFPERIOD_TICKS))
    {
      RTC.CMP = RTC_HALFPERIOD_TICKS;
    }

  return 0;
}

/****************************************************************************
 * Function:  avrdx_rtc_await_nobusy
 *
 * Description:
 *   RTC peripheral runs asynchronously from main clock, writes to some
 *   registers take time to synchronize and subsequent writes will
 *   not take effect.
 *
 *   This method reads RTC.STATUS and spins until the requested register
 *   isn't busy.
 *
 *   If instructed, it will also enable and disable interrupts while
 *   spinning so at least something can run, this is otherwise quite
 *   multitasking-unfriendly
 *
 ****************************************************************************/

static void avrdx_rtc_await_nobusy(uint8_t wait_flag, irqstate_t irqstate)
{
  while (RTC.STATUS & wait_flag)
    {
      up_irq_restore(irqstate); /* Might not actually enable interrupts
                                 * based on irqstate contents */
      asm volatile("nop");      /* Why not, we are waiting for 2 ticks
                                 * of 32768Hz clock anyway */
      up_irq_save();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the system timer.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  irqstate_t irqstate;

  /* Setup RTC timer
   */

  irqstate = up_irq_save();

  g_avrdx_system_timer_s.uptime.tv_sec = 0;
  g_avrdx_system_timer_s.uptime.tv_nsec = 0;
  g_avrdx_system_timer_s.alarm.tv_sec = 0;
  g_avrdx_system_timer_s.alarm.tv_nsec = 0;
  g_avrdx_system_timer_s.last_cnt = 0;
  g_avrdx_system_timer_s.active = 0;
  g_avrdx_system_timer_s.low_freq_prescaler = 0;
  g_avrdx_system_timer_s.cmp_is_set = 0;
  g_avrdx_system_timer_s.cmp_intr_ran_last = 0;

  /* RTC initialization procedure as per docs:
   * 1. Configure desired oscillator in Clock Controller
   *    (Using OSC32K which is preconfigured. Done.)
   * 2. Write RTC.CLKSEL clock select field.
   *    (Using OSC32K which is preconfigured. Done.)
   * 3. Write compare and period registers.
   */

  /* Use maximum period available
   */

  avrdx_rtc_await_nobusy(RTC_PERBUSY_bm,
                         irqstate);

  /* From the docs: it (the RTC) will generate ... an overflow
   * interrupt and/or event at the first count after the counter
   * value equals the Period register value. The overflow will
   * reset the counter value to zero.
   *
   * This means that when OVF interrupt executes, RTC.CNT reads 0
   * (unless the handler is delayed) regardless of PER setting.
   * Before that, it can read value of 65535 though.
   */

  RTC.PER = 65535; /* 4s interval with DIV2 */

  /* Pre-set compare match to half of the period. Interrupt
   * for compare match executes when the counter counts above
   * value set here.
   *
   * IOW if the interrupt handler runs immediately, it will
   * read 32768 from RTC.CNT (tested)
   */

  avrdx_rtc_await_nobusy(RTC_CMPBUSY_bm,
                         irqstate);
  RTC.CMP = RTC_HALFPERIOD_TICKS;

  /* 4. Enable desired interrupts */

  RTC.INTCTRL |= (RTC_OVF_bm | RTC_CMP_bm);

  /* 5. Configure prescaler */

  avrdx_rtc_await_nobusy(RTC_CTRLABUSY_bm,
                         irqstate);
  RTC.CTRLA = (RTC.CTRLA & ~RTC_PRESCALER_GM) | RTC_PRESCALER_DIV2_GC;

  /* 6. Enable the RTC */

  avrdx_rtc_await_nobusy(RTC_CTRLABUSY_bm,
                         irqstate);
  RTC.CTRLA |= RTC_RTCEN_bm;

  /* Grab interrupt handler for the RTC peripheral */

  irq_attach(AVRDX_IRQ_RTC_CNT, (xcpt_t)avrdx_timerisr, NULL);

  up_irq_restore(irqstate);
}

/****************************************************************************
 * Function:  up_timer_gettime
 *
 * Description:
 *   Updates current uptime and returns it to the caller.
 *
 ****************************************************************************/

int up_timer_gettime(FAR struct timespec *ts)
{
  irqstate_t irqstate;
  uint8_t context;
  uint8_t temp;

  irqstate = up_irq_save();

  /* Temporary variable to make nxstyle happy. */

  temp = CPU_I_bm;
  context = (((uint8_t)irqstate) & temp) ? 0 : NO_INTERRUPT_ENABLE;
  avrdx_increment_uptime(context);

  /* Better safe than sorry - do the NULL check */

  if (ts)
    {
      ts->tv_sec = g_avrdx_system_timer_s.uptime.tv_sec;
      ts->tv_nsec = g_avrdx_system_timer_s.uptime.tv_nsec;
    }

  up_irq_restore(irqstate);
  return 0;
}

/****************************************************************************
 * Function:  up_alarm_cancel
 *
 * Description:
 *   Cancels previously set alarm. See the porting guide to obtain more
 *   information on what this method is supposed to to
 *
 ****************************************************************************/

int up_alarm_cancel(FAR struct timespec *ts)
{
  irqstate_t irqstate;

  irqstate = up_irq_save();

  /* Even if the alarm already fired, it will not be reported as such.
   * "If, as a race condition, the alarm has already expired when this
   * function is called, then time returned is the current time."
   */

  avrdx_deactivate_alarm();
  avrdx_increment_uptime(NO_INTERRUPT_ENABLE);

  if (ts)
    {
      up_timer_gettime(ts);
    }

  up_irq_restore(irqstate);
  return 0;
}

/****************************************************************************
 * Function:  up_alarm_start
 *
 * Description:
 *   Sets an alarm, requests that NuttX be notified when the uptime reaches
 *   provided value. See the porting guide to obtain more information on what
 *   this method is supposed to to
 *
 ****************************************************************************/

int up_alarm_start(FAR const struct timespec *ts)
{
  irqstate_t irqstate;

  irqstate = up_irq_save();
  g_avrdx_system_timer_s.active = 1;

  /* This flag is usually cleared in this situation - unless this call
   * overrides already existing alarm. In that case, having the flag set
   * would prevent setting CMP correctly.
   */

  g_avrdx_system_timer_s.cmp_is_set = 0;

  g_avrdx_system_timer_s.alarm.tv_sec = ts->tv_sec;
  g_avrdx_system_timer_s.alarm.tv_nsec = ts->tv_nsec;

  /* tv_nsec is defined as long, can be negative, filter that out */

  if (g_avrdx_system_timer_s.alarm.tv_nsec < 0)
    {
      if (g_avrdx_system_timer_s.alarm.tv_sec)
        {
          g_avrdx_system_timer_s.alarm.tv_sec -= 1;
          g_avrdx_system_timer_s.alarm.tv_nsec += 1000ul * 1000 * 1000;
        }
      else
        {
          /* The alarm time is negative altogether ?? */

          g_avrdx_system_timer_s.alarm.tv_nsec = 0;
        }
    }

  avrdx_increment_uptime(NO_INTERRUPT_ENABLE);

  /* Call to avrdx_increment_uptime internally calls
   * avrdx_check_alarm_expired but only if the current time
   * differs from last known time. We just updated last known
   * time, force the check.
   */

  avrdx_check_alarm_expired(NO_INTERRUPT_ENABLE);

  up_irq_restore(irqstate);

  return 0;
}

#ifndef CONFIG_SCHED_TICKLESS_ALARM
#  warning Using timer API is pretty much unstested, set SCHED_TICKLESS_ALARM

/****************************************************************************
 * Function:  up_timer_cancel
 *
 * Description:
 *   Timer API for the tickless OS. Cancels previously set alarm.
 *   See the porting guide to obtain more information on what this method
 *   is supposed to to. Note that a) internally this just translates to alarm
 *   API and b) it is pretty much untested
 *
 ****************************************************************************/

int up_timer_cancel(FAR struct timespec *ts)
{
  irqstate_t irqstate;
  struct timespec temp_ts;

  /* Shortcut variables */

  register struct timespec *uptime;
  register struct timespec *alarm;

  irqstate = up_irq_save();

  /* Again, better safe then sorry */

  if (ts)
    {
      ts->tv_sec = 0;
      ts->tv_nsec = 0;
    }

  if (!g_avrdx_system_timer_s.active)
    {
      up_irq_restore(irqstate);
      return 0;
    }

  up_alarm_cancel(0);

  if (!ts)
    {
      up_irq_restore(irqstate);
      return 0;
    }

  uptime = &(g_avrdx_system_timer_s.uptime);
  alarm = &(g_avrdx_system_timer_s.alarm);

  if (alarm->tv_sec < uptime->tv_sec)
    {
      up_irq_restore(irqstate);
      return 0;
    }

  temp_ts.tv_sec = alarm.tv_sec - uptime->tv_sec;
  if (alarm->tv_nsec < uptime->tv_nsec)
    {
      if (!(temp_ts.tv_sec))
        {
          up_irq_restore(irqstate);
          return 0;
        }

      temp_ts.tv_sec--;
      temp_ts.tv_nsec = alarm->tv_nsec + 1000ul * 1000 * 1000;
      temp_ts.tv_nsec = uptime->tv_nsec;
    }
  else
    {
      temp_ts.tv_nsec = alarm->tv_nsec - uptime->tv_nsec;
    }

  ts->tv_sec = temp_ts.tv_sec;
  ts->tv_nsec = temp_ts.tv_nsec;

  up_irq_restore(irqstate);

  return 0;
}

/****************************************************************************
 * Function:  up_timer_cancel
 *
 * Description:
 *   Timer API for the tickless OS. Sets an alarm by specifying the time
 *   interval from now. See the porting guide to obtain more information
 *   on what this method is supposed to to. Note that a) internally this
 *   just translates to alarm API and b) it is pretty much untested
 *
 ****************************************************************************/

int up_timer_start(FAR const struct timespec *ts)
{
  irqstate_t irqstate;
  struct timespec temp_ts;

  irqstate = up_irq_save();

  temp_ts.tv_nsec = ts->tv_nsec;
  temp_ts.tv_sec = ts->tv_sec;

  avrdx_increment_uptime(0);

  temp_ts.tv_nsec += g_avrdx_system_timer_s.uptime.tv_nsec;
  temp_ts.tv_sec += g_avrdx_system_timer_s.uptime.tv_sec;

  if (temp_ts.tv_nsec >= 1000ul * 1000 * 1000)
    {
      temp_ts.tv_nsec -= 1000ul * 1000 * 1000;
      temp_ts.tv_sec += 1;
    }

  up_alarm_start(&temp_ts);

  up_irq_restore(irqstate);

  return 0;
}
#endif
