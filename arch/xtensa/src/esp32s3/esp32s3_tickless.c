/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_tickless.c
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
#include <arch/board/board.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "chip.h"
#include "esp32s3_irq.h"
#include "hardware/esp32s3_systimer.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_soc.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S3_SYSTIMER_TICKS_PER_SEC  (16 * 1000 * 1000)

#define CTICK_PER_SEC         (ESP32S3_SYSTIMER_TICKS_PER_SEC)
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

static inline uint64_t tickless_getcounter(void);
static inline uint64_t tickless_getalarmvalue(void);
static void IRAM_ATTR tickless_setcounter(uint64_t ticks);
static int IRAM_ATTR tickless_isr(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_timer_started; /* Whether an interval timer is being started */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tickless_getcounter
 *
 * Description:
 *   Return the total ticks of system since power-on.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Total system ticks.
 *
 ****************************************************************************/

static inline uint64_t tickless_getcounter(void)
{
  uint32_t lo;
  uint32_t lo_start;
  uint32_t hi;
  uint64_t counter;

  /* Set the "update" bit and wait for acknowledgment */

  modifyreg32(SYSTIMER_UNIT0_OP_REG, 0, SYSTIMER_TIMER_UNIT0_UPDATE);
  while ((getreg32(SYSTIMER_UNIT0_OP_REG) &
          SYSTIMER_TIMER_UNIT0_VALUE_VALID_M) !=
         SYSTIMER_TIMER_UNIT0_VALUE_VALID_M);

  /* Read LO, HI, then LO again, check that LO returns the same value.
   * This accounts for the case when an interrupt may happen between reading
   * HI and LO values, and this function may get called from the ISR.
   * In this case, the repeated read will return consistent values.
   */

  lo_start = getreg32(SYSTIMER_UNIT0_VALUE_LO_REG);
  do
    {
      lo = lo_start;
      hi = getreg32(SYSTIMER_UNIT0_VALUE_HI_REG);
      lo_start = getreg32(SYSTIMER_UNIT0_VALUE_LO_REG);
    }
  while (lo_start != lo);

  counter = ((uint64_t) hi << 32) | lo;

  return counter;
}

/****************************************************************************
 * Name: tickless_getalarmvalue
 *
 * Description:
 *   Return the remaining ticks in the currently running timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Remaining ticks.
 *
 ****************************************************************************/

static inline uint64_t tickless_getalarmvalue(void)
{
  uint32_t hi = getreg32(SYSTIMER_TARGET0_HI_REG);
  uint32_t lo = getreg32(SYSTIMER_TARGET0_LO_REG);
  uint64_t ticks = ((uint64_t) hi << 32) | lo;

  return ticks;
}

/****************************************************************************
 * Name: tickless_setcounter
 *
 * Description:
 *   Set the new value for the timer counter.
 *
 * Input Parameters:
 *   ticks - Ticks for a timer operation.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void IRAM_ATTR tickless_setcounter(uint64_t ticks)
{
  uint64_t alarm_ticks = tickless_getcounter() + ticks;

  /* Select alarm mode */

  modifyreg32(SYSTIMER_TARGET0_CONF_REG, SYSTIMER_TARGET0_PERIOD_MODE, 0);

  /* Set alarm value */

  putreg32(alarm_ticks & 0xffffffff, SYSTIMER_TARGET0_LO_REG);
  putreg32((alarm_ticks >> 32) & 0xfffff, SYSTIMER_TARGET0_HI_REG);

  /* Apply alarm value */

  putreg32(SYSTIMER_TIMER_COMP0_LOAD, SYSTIMER_COMP0_LOAD_REG);

  /* Enable alarm */

  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TARGET0_WORK_EN);

  /* Enable interrupt */

  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET0_INT_CLR);
  modifyreg32(SYSTIMER_INT_ENA_REG, 0, SYSTIMER_TARGET0_INT_ENA);
}

/****************************************************************************
 * Name: tickless_isr
 *
 * Description:
 *   Called as the IRQ handler for timer expiration.
 *
 * Input Parameters:
 *   irq           - CPU interrupt index.
 *   context       - Context data from the ISR.
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int IRAM_ATTR tickless_isr(int irq, void *context, void *arg)
{
  g_timer_started = false;

  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET0_INT_CLR);

  nxsched_timer_expiration();

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

  ticks = tickless_getcounter();
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
  uint64_t alarm_value;
  uint64_t counter;
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
          alarm_value = tickless_getalarmvalue();
          counter = tickless_getcounter();
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

  modifyreg32(SYSTIMER_CONF_REG, SYSTIMER_TARGET0_WORK_EN, 0);
  modifyreg32(SYSTIMER_INT_ENA_REG, SYSTIMER_TARGET0_INT_ENA, 0);
  modifyreg32(SYSTIMER_INT_CLR_REG, SYSTIMER_TARGET0_INT_CLR, 0);

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

  tickless_setcounter(cpu_ticks);
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

  cpuint = esp32s3_setup_irq(0, ESP32S3_PERIPH_SYSTIMER_TARGET0, 1,
                             ESP32S3_CPUINT_LEVEL);

  DEBUGASSERT(cpuint >= 0);

  /* Attach the timer interrupt. */

  irq_attach(ESP32S3_IRQ_SYSTIMER_TARGET0, tickless_isr, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(ESP32S3_IRQ_SYSTIMER_TARGET0);

  /* Enable timer clock */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_SYSTIMER_CLK_EN);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SYSTIMER_RST, 0);
  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_CLK_EN);

  /* Stall systimer 0 when CPU stalls, e.g., when using JTAG to debug */

  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT0_CORE0_STALL_EN);
#ifdef CONFIG_SMP
  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT0_CORE1_STALL_EN);
#endif
}

#endif /* CONFIG_SCHED_TICKLESS */
