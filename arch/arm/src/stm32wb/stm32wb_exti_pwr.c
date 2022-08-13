/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_exti_pwr.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_gpio.h"
#include "stm32wb_exti.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the PVD EXTI */

static xcpt_t g_pvd_callback;
static void  *g_callback_arg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_exti_pvd_isr
 *
 * Description:
 *   EXTI PVD interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32wb_exti_pvd_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending EXTI interrupt */

  putreg32(EXTI_PR1_PIF(EXTI_EVT_PVD), STM32WB_EXTI_PR1);

  /* And dispatch the interrupt to the handler */

  if (g_pvd_callback != NULL)
    {
      ret = g_pvd_callback(irq, context, g_callback_arg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_exti_pvd
 *
 * Description:
 *   Sets/clears EXTI PVD interrupt.
 *
 * Input Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edge
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int stm32wb_exti_pvd(bool risingedge, bool fallingedge, bool event,
                     xcpt_t func, void *arg)
{
  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

  g_pvd_callback = func;
  g_callback_arg = arg;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      irq_attach(STM32WB_IRQ_PVD, stm32wb_exti_pvd_isr, NULL);
      up_enable_irq(STM32WB_IRQ_PVD);
    }
  else
    {
      up_disable_irq(STM32WB_IRQ_PVD);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32WB_EXTI_RTSR1,
              risingedge ? 0 : EXTI_RTSR1_RT(EXTI_EVT_PVD),
              risingedge ? EXTI_RTSR1_RT(EXTI_EVT_PVD) : 0);
  modifyreg32(STM32WB_EXTI_FTSR1,
              fallingedge ? 0 : EXTI_FTSR1_FT(EXTI_EVT_PVD),
              fallingedge ? EXTI_FTSR1_FT(EXTI_EVT_PVD) : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32WB_EXTI_C1EMR1,
              event ? 0 : EXTI_C1EMR1_EM(EXTI_EVT_PVD),
              event ? EXTI_C1EMR1_EM(EXTI_EVT_PVD) : 0);
  modifyreg32(STM32WB_EXTI_C1IMR1,
              func ? 0 : EXTI_C1IMR1_IM(EXTI_EVT_PVD),
              func ? EXTI_C1IMR1_IM(EXTI_EVT_PVD) : 0);

  return OK;
}
