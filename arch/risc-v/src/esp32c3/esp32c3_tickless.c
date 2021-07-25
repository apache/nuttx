/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_tickless.c
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
#include <arch/board/board.h>
#include <arch/irq.h>

#include "esp32c3_attr.h"
#include "chip.h"
#include "esp32c3.h"
#include "esp32c3_irq.h"
#include "hardware/esp32c3_systimer.h"
#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_soc.h"

#include "esp32c3_rtc.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_SYSTIMER_TICKS_PER_SEC  (16 * 1000 * 1000)

#define CTICK_PER_SEC         (ESP32C3_SYSTIMER_TICKS_PER_SEC)
#define CTICK_PER_USEC        (CTICK_PER_SEC / USEC_PER_SEC)

#define SEC_2_CTICK(s)        ((s) * CTICK_PER_SEC)
#define USEC_2_CTICK(us)      ((us) * CTICK_PER_USEC)
#define NSEC_2_CTICK(nsec)    (((nsec) * CTICK_PER_USEC) / NSEC_PER_USEC)

#define CTICK_2_SEC(tick)     ((tick) / CTICK_PER_SEC)
#define CTICK_2_USEC(tick)    ((tick) / CTICK_PER_USEC)
#define CTICK_2_NSEC(tick)    ((tick) * 1000 / CTICK_PER_USEC)

#define CPU_TICKS_MAX         (UINT32_MAX / 4 * 3)

/* The structure of the counter value in systimer */

struct systimer_counter_value_s
{
  union
    {
      struct
        {
          uint64_t lo : SOC_SYSTIMER_BIT_WIDTH_LO; /* Low part of counter value */
          uint64_t hi : SOC_SYSTIMER_BIT_WIDTH_HI; /* High part of counter value */
        };
      uint64_t val; /* counter value */
    };
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint64_t up_tmr_getcounter(void);
static inline uint64_t up_tmr_getalarmvalue(void);
static inline void up_tmr_counter_advance(uint64_t tick);
static void IRAM_ATTR up_tmr_setcounter(uint64_t ticks);
static void IRAM_ATTR up_timer_expire(int irq, void *regs, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_timer_started; /* Whether an interval timer is being started */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_tmr_getcounter
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

static inline uint64_t up_tmr_getcounter(void)
{
  uint32_t lo;
  uint32_t lo_start;
  uint32_t hi;

  /* Set the "update" bit and wait for acknowledgment */

  REG_SET_BIT(SYS_TIMER_SYSTIMER_UNIT0_OP_REG, 1 << 30);
  while (!REG_GET_BIT(SYS_TIMER_SYSTIMER_UNIT0_OP_REG, 1 << 29));

  /* Read LO, HI, then LO again, check that LO returns the same value.
   * This accounts for the case when an interrupt may happen between reading
   * HI and LO values, and this function may get called from the ISR.
   * In this case, the repeated read will return consistent values.
   */

  lo_start = getreg32(SYS_TIMER_SYSTIMER_UNIT0_VALUE_HI_REG);
  do
    {
      lo = lo_start;
      hi = getreg32(SYS_TIMER_SYSTIMER_UNIT0_VALUE_HI_REG);
      lo_start = getreg32(SYS_TIMER_SYSTIMER_UNIT0_VALUE_LO_REG);
    }
  while (lo_start != lo);

  struct systimer_counter_value_s result =
    {
      .lo = lo,
      .hi = hi
    };

  return result.val;
}

/****************************************************************************
 * Name: up_tmr_getalarmvalue
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

static inline uint64_t up_tmr_getalarmvalue(void)
{
  return ((uint64_t) getreg32(SYS_TIMER_SYSTIMER_TARGET0_HI_REG) << 32) \
           | getreg32(SYS_TIMER_SYSTIMER_TARGET0_LO_REG);
}

/****************************************************************************
 * Name: up_tmr_counter_advance
 *
 * Description:
 *   Adjust current system tick by a certain value
 *
 * Input Parameters:
 *   ticks - Adjustment to apply to system time, in microseconds
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void up_tmr_counter_advance(uint64_t tick)
{
  /* set_counter_value */

  putreg32(tick & 0xffffffff, SYS_TIMER_SYSTIMER_UNIT0_LOAD_LO_REG);
  putreg32((tick >> 32) & 0xfffff, SYS_TIMER_SYSTIMER_UNIT0_LOAD_HI_REG);

  /* apply_counter_value */

  REG_SET_BIT(SYS_TIMER_SYSTIMER_UNIT0_LOAD_REG, SYS_TIMER_TIMER_UNIT0_LOAD);
}

/****************************************************************************
 * Name: up_tmr_setcounter
 *
 * Description:
 *   Set the new value of the compare register
 *
 * Input Parameters:
 *   ticks - ticks for a timer operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR up_tmr_setcounter(uint64_t ticks)
{
  uint64_t alarm_ticks = up_tmr_getcounter() + ticks;

  /* select_alarm_mode */

  REG_CLR_BIT(SYS_TIMER_SYSTIMER_TARGET0_CONF_REG,
              SYS_TIMER_TARGET0_PERIOD_MODE);

  /* set alarm value */

  putreg32(alarm_ticks & 0xffffffff, SYS_TIMER_SYSTIMER_TARGET0_LO_REG);
  putreg32((alarm_ticks >> 32) & 0xfffff, SYS_TIMER_SYSTIMER_TARGET0_HI_REG);

  /* apply alarm value */

  REG_SET_BIT(SYS_TIMER_SYSTIMER_COMP0_LOAD_REG, SYS_TIMER_TIMER_COMP0_LOAD);

  /* Enable alarm */

  REG_SET_BIT(SYS_TIMER_SYSTIMER_CONF_REG, SYS_TIMER_TARGET0_WORK_EN);

  /* Enable interrupt */

  REG_SET_BIT(SYS_TIMER_SYSTIMER_INT_CLR_REG, SYS_TIMER_TARGET0_INT_CLR);
  REG_SET_BIT(SYS_TIMER_SYSTIMER_INT_ENA_REG, SYS_TIMER_TARGET0_INT_ENA);
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

static void IRAM_ATTR up_timer_expire(int irq, void *regs, FAR void *arg)
{
  g_timer_started = false;
  setbits(SYS_TIMER_TARGET0_INT_CLR, SYS_TIMER_SYSTIMER_INT_CLR_REG);
  nxsched_timer_expiration();
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

int IRAM_ATTR up_timer_gettime(FAR struct timespec *ts)
{
  uint64_t ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  ticks = up_tmr_getcounter();
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

int IRAM_ATTR up_timer_cancel(FAR struct timespec *ts)
{
  uint64_t alarm_value;
  uint64_t counter;
  irqstate_t flags;

  flags = enter_critical_section();

  if (ts != NULL)
    {
      if (g_timer_started == false)
        {
          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          alarm_value = up_tmr_getalarmvalue();
          counter = up_tmr_getcounter();
          if (alarm_value <= counter)
            {
              alarm_value = 0;
            }
          else
            {
              alarm_value -= counter;
            }

          ts->tv_sec  = CTICK_2_SEC(alarm_value);
          ts->tv_nsec = CTICK_2_NSEC(alarm_value % CTICK_PER_SEC);
        }
    }

  g_timer_started = false;
  REG_CLR_BIT(SYS_TIMER_SYSTIMER_CONF_REG, SYS_TIMER_TARGET0_WORK_EN);
  REG_CLR_BIT(SYS_TIMER_SYSTIMER_INT_ENA_REG, SYS_TIMER_TARGET0_INT_ENA);
  REG_SET_BIT(SYS_TIMER_SYSTIMER_INT_CLR_REG, SYS_TIMER_TARGET0_INT_CLR);

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

int IRAM_ATTR up_timer_start(FAR const struct timespec *ts)
{
  uint64_t cpu_ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  if (g_timer_started == true)
    {
      up_timer_cancel(NULL);
    }

  cpu_ticks = SEC_2_CTICK((uint64_t)ts->tv_sec) +
              NSEC_2_CTICK((uint64_t)ts->tv_nsec);

  up_tmr_setcounter(cpu_ticks);
  g_timer_started = true;

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
  int cpuint;

  g_timer_started = false;

  /* Enable timer clock */

  setbits(SYSTEM_SYSTIMER_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
  resetbits(SYSTEM_SYSTIMER_RST, SYSTEM_PERIP_RST_EN0_REG);

  setbits(SYS_TIMER_CLK_EN, SYS_TIMER_SYSTIMER_CONF_REG);

  /* Stall timer when stall CPU, specially when using JTAG to debug */

  setbits(SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN, SYS_TIMER_SYSTIMER_CONF_REG);

  /* NOTE: Timer 0 is an internal interrupt source so we do not need to
   * attach any peripheral ID to the dedicated CPU interrupt.
   */

  /* Attach the timer interrupt */

  cpuint = esp32c3_request_irq(ESP32C3_PERIPH_SYSTIMER_T0,
                               ESP32C3_INT_PRIO_DEF,
                               ESP32C3_INT_LEVEL);

  /* Attach the timer interrupt. */

  irq_attach(ESP32C3_IRQ_SYSTIMER_T0, (xcpt_t)up_timer_expire, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(cpuint);
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
  uint64_t alarm_value;
  uint64_t counter;
  irqstate_t flags;

  flags = enter_critical_section();
  if (g_timer_started == false)
    {
      us = 0;
    }
  else
    {
      alarm_value = up_tmr_getalarmvalue();
      counter = up_tmr_getcounter();
      if (alarm_value > counter)
        {
          us = CTICK_2_USEC(alarm_value - counter);
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
  uint64_t step_counter;
  uint64_t alarm_value;
  irqstate_t flags;

  DEBUGASSERT(g_timer_started);

  flags = enter_critical_section();

  alarm_value = up_tmr_getalarmvalue();
  step_counter = USEC_2_CTICK((uint64_t)us) + up_tmr_getcounter();
  if (step_counter > alarm_value)
    {
      DEBUGASSERT(0);
    }

  up_tmr_counter_advance(step_counter);

  leave_critical_section(flags);
}

#endif /* CONFIG_SCHED_TICKLESS */