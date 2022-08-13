/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_timerisr.c
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
#include <nuttx/arch.h>

#include "chip.h"
#include "esp32s3_irq.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_systimer.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32S3_SYSTIMER_TICKS_PER_SEC  (16 * 1000 * 1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: systimer_isr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
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

static int systimer_isr(int irq, void *context, void *arg)
{
  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET0_INT_CLR);

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
  uint32_t regval;
  int cpuint;

  cpuint = esp32s3_setup_irq(0, ESP32S3_PERIPH_SYSTIMER_TARGET0, 1,
                             ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(cpuint >= 0);

  /* Attach the timer interrupt. */

  irq_attach(ESP32S3_IRQ_SYSTIMER_TARGET0, systimer_isr, NULL);

  /* Enable the allocated CPU interrupt. */

  up_enable_irq(ESP32S3_IRQ_SYSTIMER_TARGET0);

  /* Enable timer clock */

  modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_SYSTIMER_CLK_EN);
  modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SYSTIMER_RST, 0);
  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_CLK_EN);

  /* Configure alarm 0 (Comparator 0) */

  regval = SYSTIMER_TARGET0_PERIOD_MODE |
           ((ESP32S3_SYSTIMER_TICKS_PER_SEC / CLOCKS_PER_SEC) <<
            SYSTIMER_TARGET0_PERIOD_S);
  putreg32(regval, SYSTIMER_TARGET0_CONF_REG);
  putreg32(SYSTIMER_TIMER_COMP0_LOAD, SYSTIMER_COMP0_LOAD_REG);

  /* Stall systimer 0 when CPU stalls, e.g., when using JTAG to debug */

  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT0_CORE0_STALL_EN);
#ifdef CONFIG_SMP
  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT0_CORE1_STALL_EN);
#endif

  /* Enable interrupt */

  modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_TARGET0_INT_CLR);
  modifyreg32(SYSTIMER_INT_ENA_REG, 0, SYSTIMER_TARGET0_INT_ENA);

  /* Start alarm 0 and counter 0 */

  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TARGET0_WORK_EN);
  modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_TIMER_UNIT0_WORK_EN);
}
