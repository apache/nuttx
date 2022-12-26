/****************************************************************************
 * arch/xtensa/src/esp32/esp32_tickless.c
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
 *   void sched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <arch/xtensa/xtensa_specregs.h>
#include <arch/xtensa/core_macros.h>
#include <arch/board/board.h>

#include "xtensa_timer.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "xtensa_counter.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CTICK_PER_SEC         (BOARD_CLOCK_FREQUENCY)
#define CTICK_PER_USEC        (CTICK_PER_SEC / USEC_PER_SEC)

#define SEC_2_CTICK(s)        ((s) * CTICK_PER_SEC)
#define USEC_2_CTICK(us)      ((us) * CTICK_PER_USEC)
#define NSEC_2_CTICK(nsec)    (((nsec) * CTICK_PER_USEC) / NSEC_PER_USEC)

#define CTICK_2_SEC(tick)     ((tick) / CTICK_PER_SEC)
#define CTICK_2_USEC(tick)    ((tick) / CTICK_PER_USEC)
#define CTICK_2_NSEC(tick)    ((tick) * 1000 / CTICK_PER_USEC)

#define CPU_TICKS_MAX         (UINT32_MAX / 4 * 3)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint64_t up_tmr_total_count(void);
static inline uint64_t up_tmr_getcount(void);
static void IRAM_ATTR up_tmr_setcompare(uint32_t ticks);
static void IRAM_ATTR up_tmr_setcount(uint64_t ticks);
static int up_timer_expire(int irq, void *regs, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_timer_started; /* Whether an interval timer is being started */
static uint64_t g_cticks;    /* Total ticks of system since power-on */
static uint32_t g_loop_cnt;  /* System Cycle counter cycle times */

/* Redundant ticks of an interval timer on the cycle counter */

static uint32_t g_last_cticks;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_tmr_total_count
 *
 * Description:
 *   Return the total ticks of system since power-on.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Total system ticks
 *
 ****************************************************************************/

static inline uint64_t up_tmr_total_count(void)
{
  return g_cticks + xtensa_getcount();
}

/****************************************************************************
 * Name: up_tmr_getcount
 *
 * Description:
 *   Return the remaining ticks in the currently running timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Remaining ticks
 *
 ****************************************************************************/

static inline uint64_t up_tmr_getcount(void)
{
  return (uint64_t)g_loop_cnt * CPU_TICKS_MAX +
         g_last_cticks;
}

/****************************************************************************
 * Name: up_tmr_setcompare
 *
 * Description:
 *   Set the value of the compare register, save the currently running
 *   system tick and clear cycle count register.
 *
 * Input Parameters:
 *   ticks - Set the new value of the compare register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR up_tmr_setcompare(uint32_t ticks)
{
  xtensa_setcompare(ticks);
  g_cticks += xtensa_getcount();
  xtensa_setcount(0);
}

/****************************************************************************
 * Name: up_tmr_setcount
 *
 * Description:
 *   Set the value of the compare register
 *
 * Input Parameters:
 *   ticks - ticks for a timer operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR up_tmr_setcount(uint64_t ticks)
{
  irqstate_t flags;
  uint32_t loop_cnt;
  uint32_t last_ticks;

  if (ticks == 0)
    {
      ticks = 1;
    }

  loop_cnt   = ticks / CPU_TICKS_MAX;
  last_ticks = ticks % CPU_TICKS_MAX;

  if (loop_cnt != 0)
    {
      xtensa_setcompare(CPU_TICKS_MAX);
    }
  else
    {
      xtensa_setcompare(last_ticks);
    }

  flags = enter_critical_section();

  g_loop_cnt      = loop_cnt;
  g_last_cticks   = last_ticks;
  g_timer_started = true;

  leave_critical_section(flags);

  g_cticks += xtensa_getcount();
  xtensa_setcount(0);
}

/****************************************************************************
 * Name: up_timer_expire
 *
 * Description:
 *   Called as the IRQ handler for timer expiration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int up_timer_expire(int irq, void *regs, void *arg)
{
  irqstate_t flags;
  bool do_sched = false;

  flags = enter_critical_section();

  if (g_timer_started)
    {
      if (g_loop_cnt != 0)
        {
          --g_loop_cnt;
          if (g_loop_cnt == 0)
            {
              if (g_last_cticks != 0)
                {
                  up_tmr_setcompare(g_last_cticks);
                }
              else
                {
                  do_sched = true;
                  up_tmr_setcompare(CPU_TICKS_MAX);
                }
            }
          else
            {
              up_tmr_setcompare(CPU_TICKS_MAX);
            }
        }
      else
        {
          do_sched = true;
        }

      if (do_sched)
        {
          up_timer_cancel(NULL);
          nxsched_timer_expiration();
        }
    }
  else
    {
      up_tmr_setcompare(CPU_TICKS_MAX);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int IRAM_ATTR up_timer_gettime(struct timespec *ts)
{
  uint64_t ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  ticks = up_tmr_total_count();
  ts->tv_sec  = CTICK_2_SEC(ticks);
  ts->tv_nsec = CTICK_2_NSEC(ticks % CTICK_PER_SEC);

  leave_critical_section(flags);
  return OK;
}

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

int IRAM_ATTR up_timer_cancel(struct timespec *ts)
{
  uint64_t rst_ticks;
  uint64_t cur_ticks;
  uint64_t ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  if (ts != NULL)
    {
      if (!g_timer_started)
        {
          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          rst_ticks = up_tmr_getcount();
          cur_ticks = xtensa_getcount();
          if (rst_ticks <= cur_ticks)
            {
              ticks = 0;
            }
          else
            {
              ticks = rst_ticks - cur_ticks;
            }

          ts->tv_sec  = CTICK_2_SEC(ticks);
          ts->tv_nsec = CTICK_2_NSEC(ticks % CTICK_PER_SEC);
        }
    }

  g_timer_started = false;
  up_tmr_setcompare(CPU_TICKS_MAX);

  leave_critical_section(flags);
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

int IRAM_ATTR up_timer_start(const struct timespec *ts)
{
  uint64_t cpu_ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_timer_started)
    {
      up_timer_cancel(NULL);
    }

  cpu_ticks = SEC_2_CTICK((uint64_t)ts->tv_sec) +
              NSEC_2_CTICK((uint64_t)ts->tv_nsec);

  up_tmr_setcount(cpu_ticks);

  leave_critical_section(flags);
  return OK;
}

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
  /* Set up periodic timer */

  up_tmr_setcompare(CPU_TICKS_MAX);

  /* NOTE: Timer 0 is an internal interrupt source so we do not need to
   * attach any peripheral ID to the dedicated CPU interrupt.
   */

  /* Attach the timer interrupt */

  irq_attach(XTENSA_IRQ_TIMER0, (xcpt_t)up_timer_expire, NULL);

  /* Enable the timer 0 CPU interrupt. */

  up_enable_irq(XTENSA_IRQ_TIMER0);
}

/****************************************************************************
 * Name: up_get_idletime
 *
 * Description:
 *   This function returns the idle time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The time in system ticks remaining for idle.
 *   Zero means system is busy.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR up_get_idletime(void)
{
  uint32_t us;
  uint64_t ticks;
  uint64_t rst_ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  if (!g_timer_started)
    {
      us = 0;
    }
  else
    {
      ticks = xtensa_getcount();
      rst_ticks = up_tmr_getcount();
      if (rst_ticks > ticks)
        {
          us = CTICK_2_USEC(rst_ticks - ticks);
        }
      else
        {
          us = 0;
        }
    }

  leave_critical_section(flags);

  return us;
}

/****************************************************************************
 * Name:  up_step_idletime
 *
 * Description:
 *   Add system time by idletime_us.
 *
 * Input Parameters:
 *   us - Idle time(us)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR up_step_idletime(uint32_t us)
{
  uint64_t ticks;
  uint64_t timer_ticks;
  irqstate_t flags;

  DEBUGASSERT(g_timer_started);

  flags = enter_critical_section();

  ticks = USEC_2_CTICK((uint64_t)us);
  timer_ticks = up_tmr_getcount();

  DEBUGASSERT(ticks < timer_ticks);

  g_cticks += ticks;
  up_tmr_setcount(timer_ticks - ticks);

  leave_critical_section(flags);
}

#endif /* CONFIG_SCHED_TICKLESS */
