/****************************************************************************
 * arch/risc-v/src/espressif/esp_tickless.c
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

#include <assert.h>
#include <stdint.h>
#include <time.h>

#include <arch/board/board.h>
#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>

#include "chip.h"
#include "esp_irq.h"

#include "esp_attr.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"
#include "periph_ctrl.h"
#include "systimer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if SOC_SYSTIMER_INT_LEVEL
#  define SYSTIMER_TRIGGER_TYPE ESP_IRQ_TRIGGER_LEVEL
#else
#  define SYSTIMER_TRIGGER_TYPE ESP_IRQ_TRIGGER_EDGE
#endif /* SOC_SYSTIMER_INT_LEVEL */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Systimer HAL layer object */

static systimer_hal_context_t systimer_hal;

/* Whether an interval timer is being started */

static bool g_timer_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_tickless_isr
 *
 * Description:
 *   Handler to be executed by the Systimer ISR.
 *
 * Input Parameters:
 *   irq           - IRQ associated to that interrupt.
 *   context       - Interrupt register state save info.
 *   arg           - A pointer to the argument provided when the interrupt
 *                   was registered.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_tickless_isr(int irq, void *context, void *arg)
{
  g_timer_started = false;

  systimer_ll_clear_alarm_int(systimer_hal.dev,
                              SYSTIMER_ALARM_OS_TICK_CORE0);

  nxsched_timer_expiration();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_get_idletime
 *
 * Description:
 *   This function returns the idle time.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The time in system ticks remaining for idle.
 *   Zero means system is busy.
 *
 ****************************************************************************/

uint32_t up_get_idletime(void)
{
  uint32_t us;
  uint64_t alarm_value;
  uint64_t counter;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);
  if (!g_timer_started)
    {
      spin_unlock_irqrestore(NULL, flags);

      return 0;
    }

  alarm_value = systimer_hal_get_alarm_value(&systimer_hal,
                                             SYSTIMER_ALARM_OS_TICK_CORE0);
  counter = systimer_hal_get_counter_value(&systimer_hal,
                                           SYSTIMER_COUNTER_OS_TICK);
  if (alarm_value > counter)
    {
      us = systimer_hal.ticks_to_us(alarm_value - counter);
    }
  else
    {
      us = 0;
    }

  spin_unlock_irqrestore(NULL, flags);

  return us;
}

/****************************************************************************
 * Name:  up_step_idletime
 *
 * Description:
 *   Add system time by idletime_us.
 *
 * Input Parameters:
 *   idletime_us   - Idle time in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_step_idletime(uint32_t idletime_us)
{
  irqstate_t flags;

  DEBUGASSERT(g_timer_started);

  flags = spin_lock_irqsave(NULL);

  systimer_hal_counter_value_advance(&systimer_hal, SYSTIMER_COUNTER_OS_TICK,
                                     idletime_us);

  spin_unlock_irqrestore(NULL, flags);
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

int IRAM_ATTR up_timer_gettime(struct timespec *ts)
{
  uint64_t time_us;
  irqstate_t flags = spin_lock_irqsave(NULL);

  time_us = systimer_hal_get_time(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);
  ts->tv_sec  = time_us / USEC_PER_SEC;
  ts->tv_nsec = (time_us % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

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
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (ts != NULL)
    {
      if (!g_timer_started)
        {
          ts->tv_sec  = 0;
          ts->tv_nsec = 0;
        }
      else
        {
          uint64_t alarm_ticks;
          uint64_t alarm_us;
          uint64_t counter;
          uint64_t ticks_mod;

          counter = systimer_hal_get_counter_value(&systimer_hal,
                                                   SYSTIMER_COUNTER_OS_TICK);
          alarm_ticks = systimer_hal_get_alarm_value(&systimer_hal,
                                               SYSTIMER_ALARM_OS_TICK_CORE0);

          if (alarm_ticks <= counter)
            {
              alarm_ticks = 0;
            }
          else
            {
              alarm_ticks -= counter;
            }

          alarm_us = systimer_hal.ticks_to_us(alarm_ticks);
          ticks_mod = (alarm_ticks - systimer_hal.us_to_ticks(alarm_us));

          ts->tv_sec  = alarm_us / USEC_PER_SEC;
          ts->tv_nsec = systimer_hal.ticks_to_us(ticks_mod) * NSEC_PER_USEC;
        }
    }

  g_timer_started = false;

  systimer_ll_enable_alarm(systimer_hal.dev,
                           SYSTIMER_ALARM_OS_TICK_CORE0, false);
  systimer_ll_enable_alarm_int(systimer_hal.dev,
                               SYSTIMER_ALARM_OS_TICK_CORE0, false);
  systimer_ll_clear_alarm_int(systimer_hal.dev,
                              SYSTIMER_ALARM_OS_TICK_CORE0);

  spin_unlock_irqrestore(NULL, flags);

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
  uint64_t target_us;
  uint64_t alarm_ticks;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (g_timer_started)
    {
      up_timer_cancel(NULL);
    }

  target_us = (uint64_t)ts->tv_sec * USEC_PER_SEC +
                (uint64_t)(ts->tv_nsec / NSEC_PER_USEC);

  alarm_ticks  = systimer_hal_get_counter_value(&systimer_hal,
                                                SYSTIMER_COUNTER_OS_TICK);
  alarm_ticks += systimer_hal.us_to_ticks(target_us);

  systimer_hal_select_alarm_mode(&systimer_hal,
                                 SYSTIMER_ALARM_OS_TICK_CORE0,
                                 SYSTIMER_ALARM_MODE_ONESHOT);
  systimer_hal_set_alarm_target(&systimer_hal,
                                SYSTIMER_ALARM_OS_TICK_CORE0,
                                systimer_hal.ticks_to_us(alarm_ticks));
  systimer_hal_enable_alarm_int(&systimer_hal,
                                SYSTIMER_ALARM_OS_TICK_CORE0);

  g_timer_started = true;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  g_timer_started = false;

  periph_module_enable(PERIPH_SYSTIMER_MODULE);
  systimer_hal_init(&systimer_hal);
  systimer_hal_tick_rate_ops_t ops =
    {
      .ticks_to_us = systimer_ticks_to_us,
      .us_to_ticks = systimer_us_to_ticks,
    };

  systimer_hal_set_tick_rate_ops(&systimer_hal, &ops);
  systimer_ll_set_counter_value(systimer_hal.dev,
                                SYSTIMER_COUNTER_OS_TICK,
                                0);
  systimer_ll_apply_counter_value(systimer_hal.dev,
                                  SYSTIMER_COUNTER_OS_TICK);
  systimer_hal_connect_alarm_counter(&systimer_hal,
                                     SYSTIMER_ALARM_OS_TICK_CORE0,
                                     SYSTIMER_COUNTER_OS_TICK);
  systimer_hal_counter_can_stall_by_cpu(&systimer_hal,
                                        SYSTIMER_COUNTER_OS_TICK, 0,
                                        true);
  systimer_hal_enable_counter(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);

  esp_setup_irq(SYSTIMER_TARGET0_EDGE_INTR_SOURCE,
                ESP_IRQ_PRIORITY_DEFAULT,
                SYSTIMER_TRIGGER_TYPE);

  /* Attach the timer interrupt. */

  irq_attach(ESP_IRQ_SYSTIMER_TARGET0_EDGE, (xcpt_t)esp_tickless_isr, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(ESP_IRQ_SYSTIMER_TARGET0_EDGE);
}
