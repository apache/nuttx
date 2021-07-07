/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_timerisr.c
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
#include <assert.h>
#include <time.h>

#include <arch/board/board.h>
#include <arch/irq.h>

#include "chip.h"
#include "esp32c3.h"
#include "esp32c3_irq.h"
#include "hardware/esp32c3_systimer.h"
#include "hardware/esp32c3_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C3_SYSTIMER_TICKS_PER_SEC  (16 * 1000 * 1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: systimer_isr
 ****************************************************************************/

static int systimer_isr(int irq, FAR void *context, FAR void *arg)
{
  setbits(SYS_TIMER_TARGET0_INT_CLR, SYS_TIMER_SYSTIMER_INT_CLR_REG);

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
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;
  int cpuint;

  /* Enable timer clock */

  setbits(SYSTEM_SYSTIMER_CLK_EN, SYSTEM_PERIP_CLK_EN0_REG);
  resetbits(SYSTEM_SYSTIMER_RST, SYSTEM_PERIP_RST_EN0_REG);

  setbits(SYS_TIMER_CLK_EN, SYS_TIMER_SYSTIMER_CONF_REG);

  /* Configure alarm0 (Comparator 0) */

  regval = SYS_TIMER_TARGET0_PERIOD_MODE |
           ((ESP32C3_SYSTIMER_TICKS_PER_SEC / CLOCKS_PER_SEC) <<
            SYS_TIMER_TARGET0_PERIOD_S);
  putreg32(regval, SYS_TIMER_SYSTIMER_TARGET0_CONF_REG);

  putreg32(SYS_TIMER_TIMER_COMP0_LOAD, SYS_TIMER_SYSTIMER_COMP0_LOAD_REG);

  /* Stall systimer 0 when CPU stalls, e.g., when using JTAG to debug */

  setbits(SYS_TIMER_TIMER_UNIT0_CORE0_STALL_EN, SYS_TIMER_SYSTIMER_CONF_REG);

  /* Enable interrupt */

  setbits(SYS_TIMER_TARGET0_INT_CLR, SYS_TIMER_SYSTIMER_INT_CLR_REG);
  setbits(SYS_TIMER_TARGET0_INT_ENA, SYS_TIMER_SYSTIMER_INT_ENA_REG);

  regval = SYS_TIMER_TARGET0_WORK_EN;
  setbits(regval, SYS_TIMER_SYSTIMER_CONF_REG);

  /* Start alarm0 counter0 */

  regval = SYS_TIMER_TIMER_UNIT0_WORK_EN;
  setbits(regval, SYS_TIMER_SYSTIMER_CONF_REG);

  cpuint = esp32c3_request_irq(ESP32C3_PERIPH_SYSTIMER_T0,
                               ESP32C3_INT_PRIO_DEF,
                               ESP32C3_INT_LEVEL);

  /* Attach the timer interrupt. */

  irq_attach(ESP32C3_IRQ_SYSTIMER_T0, (xcpt_t)systimer_isr, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(cpuint);
}
