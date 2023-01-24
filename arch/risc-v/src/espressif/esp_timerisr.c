/****************************************************************************
 * arch/risc-v/src/espressif/esp_timerisr.c
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

#include "chip.h"
#include "esp_irq.h"

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: systimer_irq_handler
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

static int systimer_irq_handler(int irq, void *context, void *arg)
{
  systimer_ll_clear_alarm_int(systimer_hal.dev,
                              SYSTIMER_ALARM_OS_TICK_CORE0);

  /* Process timer interrupt */

  nxsched_process_timer();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  systimer_hal_counter_can_stall_by_cpu(&systimer_hal,
                                        SYSTIMER_COUNTER_OS_TICK, 0,
                                        false);

  systimer_hal_connect_alarm_counter(&systimer_hal,
                                     SYSTIMER_ALARM_OS_TICK_CORE0,
                                     SYSTIMER_COUNTER_OS_TICK);
  systimer_hal_set_alarm_period(&systimer_hal,
                                SYSTIMER_ALARM_OS_TICK_CORE0,
                                CONFIG_USEC_PER_TICK);
  systimer_hal_select_alarm_mode(&systimer_hal,
                                 SYSTIMER_ALARM_OS_TICK_CORE0,
                                 SYSTIMER_ALARM_MODE_PERIOD);
  systimer_hal_counter_can_stall_by_cpu(&systimer_hal,
                                        SYSTIMER_COUNTER_OS_TICK, 0,
                                        true);
  systimer_hal_enable_alarm_int(&systimer_hal,
                                SYSTIMER_ALARM_OS_TICK_CORE0);
  systimer_hal_enable_counter(&systimer_hal, SYSTIMER_COUNTER_OS_TICK);

  esp_setup_irq(SYSTIMER_TARGET0_EDGE_INTR_SOURCE,
                ESP_IRQ_PRIORITY_DEFAULT,
                SYSTIMER_TRIGGER_TYPE);

  /* Attach the timer interrupt. */

  irq_attach(ESP_IRQ_SYSTIMER_TARGET0_EDGE,
             (xcpt_t)systimer_irq_handler,
             NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(ESP_IRQ_SYSTIMER_TARGET0_EDGE);
}
