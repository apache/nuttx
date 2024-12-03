/****************************************************************************
 * arch/risc-v/src/litex/litex_tickless.c
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
#include <stdbool.h>
#include <assert.h>

#include "chip.h"

#include "riscv_internal.h"

#include "litex.h"
#include "litex_clockconfig.h"
#include "hardware/litex_timer.h"

#if defined(CONFIG_SCHED_TICKLESS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LITEX_TICK_PER_SEC         (CONFIG_LITEX_SYS_CORE_FREQ_HZ)
#define LITEX_TICK_PER_USEC        (LITEX_TICK_PER_SEC / USEC_PER_SEC)

#define SEC_2_LITEX_TICK(s)        ((s) * LITEX_TICK_PER_SEC)
#define USEC_2_LITEX_TICK(us)      ((us) * LITEX_TICK_PER_USEC)
#define NSEC_2_LITEX_TICK(nsec)    (((nsec) * LITEX_TICK_PER_USEC) / NSEC_PER_USEC)

#define LITEX_TICK_2_SEC(tick)     ((tick) / LITEX_TICK_PER_SEC)
#define LITEX_TICK_2_USEC(tick)    ((tick) / LITEX_TICK_PER_USEC)
#define LITEX_TICK_2_NSEC(tick)    ((tick) * 1000 / LITEX_TICK_PER_USEC)

static bool g_timer_started; /* Whether an interval timer is being started */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  g_timer_started = false;
  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_PENDING);
  nxsched_timer_expiration();
  return OK;
}

/****************************************************************************
 * Name: litex_get_uptime
 *
 * Description:
 *  Get the number of ticks remaining on the counter.
 *
 * Input Parameters:
 *   None
 *
 ****************************************************************************/

void litex_get_uptime(uint64_t * value)
{
  putreg32(1, LITEX_TIMER0_UPTIME_LATCH);
  *value = (uint64_t)getreg32(LITEX_TIMER0_UPTIME_U) << 32;
  *value |= getreg32(LITEX_TIMER0_UPTIME_L);
}

/****************************************************************************
 * Name: litex_get_remaining
 *
 * Description:
 *  Get the number of ticks remaining on the counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The count remaining on the timer.
 *
 ****************************************************************************/

uint32_t litex_get_remaining()
{
  putreg32(1, LITEX_TIMER0_UPDATE_VALUE);
  return getreg32(LITEX_TIMER0_VALUE);
}

/****************************************************************************
 * Name: litex_timer_cancel
 *
 * Input Parameters:
 *   None
 *
 * Description:
 *  Cancel any running one-shot timer.
 *
 ****************************************************************************/

void litex_timer_cancel()
{
  g_timer_started = false;
  putreg32(0, LITEX_TIMER0_EN);
  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_PENDING);
}

/****************************************************************************
 * Name: litex_timer_oneshot
 *
 * Description:
 *   Start timer0 in one-shot countdown mode.
 *
 * Input Parameters:
 *   ticks - The tick count to use for the one-shot timer.
 *
 ****************************************************************************/

void litex_timer_oneshot(const uint32_t ticks)
{
  g_timer_started = true;
  putreg32(ticks, LITEX_TIMER0_LOAD);
  putreg32(LITEX_TIMER0_ENABLE_BIT, LITEX_TIMER0_EN);
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

int up_timer_gettime(struct timespec *ts)
{
  uint64_t ticks;

  litex_get_uptime(&ticks);
  ts->tv_sec  = LITEX_TICK_2_SEC(ticks);
  ts->tv_nsec = LITEX_TICK_2_NSEC(ticks % LITEX_TICK_PER_SEC);

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

int up_timer_cancel(struct timespec *ts)
{
  uint64_t alarm_value;
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
          litex_timer_cancel();
          alarm_value = litex_get_remaining();
          ts->tv_sec  = LITEX_TICK_2_SEC(alarm_value);
          ts->tv_nsec = LITEX_TICK_2_NSEC(alarm_value % LITEX_TICK_PER_SEC);
        }
    }

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

int up_timer_start(const struct timespec *ts)
{
  uint64_t cpu_ticks;
  irqstate_t flags;

  flags = enter_critical_section();

  litex_timer_cancel();

  cpu_ticks = SEC_2_LITEX_TICK((uint64_t)ts->tv_sec) +
              NSEC_2_LITEX_TICK((uint64_t)ts->tv_nsec);

  DEBUGASSERT(cpu_ticks <= UINT32_MAX);

  litex_timer_oneshot(cpu_ticks);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Cancel any configuration that has been done in the bios or openSBI */

  litex_timer_cancel();

  /* Write zero to reload to ensure one-shot mode */

  putreg32(0, LITEX_TIMER0_RELOAD);

  putreg32(1 << LITEX_TIMER0_TIMEOUT_EV_OFFSET, LITEX_TIMER0_EV_ENABLE);
  irq_attach(LITEX_IRQ_TIMER0, up_timer_expire, NULL);
  up_enable_irq(LITEX_IRQ_TIMER0);
}

#endif /* CONFIG_SCHED_TICKLESS */
