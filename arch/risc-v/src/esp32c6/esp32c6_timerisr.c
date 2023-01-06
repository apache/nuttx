/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_timerisr.c
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
#include "esp32c6.h"
#include "esp32c6_irq.h"
#include "hardware/esp32c6_systimer.h"
#include "hardware/esp32c6_pcr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32C6_SYSTIMER_TICKS_PER_SEC  (16 * 1000 * 1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: systimer_isr
 ****************************************************************************/

static int systimer_isr(int irq, FAR void *context, FAR void *arg)
{
  setbits(SYSTIMER_TARGET0_INT_CLR, SYSTIMER_INT_CLR_REG);

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

  /* Enable timer clock */

  setbits(PCR_SYSTIMER_CLK_EN, PCR_SYSTIMER_CONF_REG);
  resetbits(PCR_SYSTIMER_RST_EN, PCR_SYSTIMER_CONF_REG);

  setbits(SYSTIMER_CLK_EN, SYSTIMER_CONF_REG);
  setbits(SYSTIMER_ETM_EN, SYSTIMER_CONF_REG);

  /* Configure alarm0 counter1 */

  regval = SYSTIMER_TARGET0_PERIOD_MODE |
           (1 << SYSTIMER_TARGET0_TIMER_UNIT_SEL_S) |
           ((ESP32C6_SYSTIMER_TICKS_PER_SEC / CLOCKS_PER_SEC) <<
            SYSTIMER_TARGET0_PERIOD_S);
  putreg32(regval, SYSTIMER_TARGET0_CONF_REG);

  putreg32(SYSTIMER_TIMER_COMP0_LOAD, SYSTIMER_COMP0_LOAD_REG);

  /* Stall timer when stall CPU, specially when using JTAG to debug */

  setbits(SYSTIMER_TIMER_UNIT0_CORE0_STALL_EN, SYSTIMER_CONF_REG);

  /* Enable interrupt */

  setbits(SYSTIMER_TARGET0_INT_CLR, SYSTIMER_INT_CLR_REG);
  setbits(SYSTIMER_TARGET0_INT_ENA, SYSTIMER_INT_ENA_REG);

  regval = SYSTIMER_TARGET0_WORK_EN;
  setbits(regval, SYSTIMER_CONF_REG);

  /* Start alarm0 counter1 */

  regval = SYSTIMER_TIMER_UNIT1_WORK_EN;
  setbits(regval, SYSTIMER_CONF_REG);
  esp32c6_setup_irq(ESP32C6_SYSTIMER_TARGET0_EDGE_PERIPH,
                    ESP32C6_INT_PRIO_DEF,
                    ESP32C6_INT_LEVEL);

  /* Attach the timer interrupt. */

  irq_attach(ESP32C6_IRQ_SYSTIMER_TARGET0_EDGE, (xcpt_t)systimer_isr, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(ESP32C6_IRQ_SYSTIMER_TARGET0_EDGE);
}
