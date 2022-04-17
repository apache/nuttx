/****************************************************************************
 * arch/arm/src/stm32l5/stm32l5_exti_gpio.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32l5_gpio.h"
#include "stm32l5_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  xcpt_t callback;   /* Callback entry point */
  void  *arg;        /* The argument that accompanies the callback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to each EXTI */

static struct gpio_callback_s g_gpio_handlers[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Interrupt Service Routine - Dispatcher
 ****************************************************************************/

static int stm32l5_exti0_15_isr(int irq, void *context, void *arg)
{
  int ret = OK;
  int exti;

  exti = irq - STM32L5_IRQ_EXTI0;
  DEBUGASSERT((exti >= 0) && (exti <= 15));

  /* Clear the pending interrupt for both rising and falling edges. */

  putreg32(0x0001 << exti, STM32L5_EXTI_RPR1);
  putreg32(0x0001 << exti, STM32L5_EXTI_FPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_handlers[exti].callback != NULL)
    {
      xcpt_t callback = g_gpio_handlers[exti].callback;
      void  *cbarg    = g_gpio_handlers[exti].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l5_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  pinset      - GPIO pin configuration
 *  risingedge  - Enables interrupt on rising edges
 *  fallingedge - Enables interrupt on falling edges
 *  event       - Generate event when set
 *  func        - When non-NULL, generate interrupt
 *  arg         - Argument passed to the interrupt callback
 *
 * Returned Value:
 *  Zero (OK) is returned on success, otherwise a negated errno value is
 *  returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32l5_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                         bool event, xcpt_t func, void *arg)
{
  uint32_t pin = pinset & GPIO_PIN_MASK;
  uint32_t exti = 1 << pin;
  int      irq = STM32L5_IRQ_EXTI0 + pin;

  g_gpio_handlers[pin].callback = func;
  g_gpio_handlers[pin].arg      = arg;

  /* Install external interrupt handlers */

  if (func)
    {
      irq_attach(irq, stm32l5_exti0_15_isr, NULL);
      up_enable_irq(irq);
    }
  else
    {
      up_disable_irq(irq);
    }

  /* Configure GPIO, enable EXTI line enabled if event or interrupt is
   * enabled.
   */

  if (event || func)
    {
      pinset |= GPIO_EXTI;
    }

  stm32l5_configgpio(pinset);

  /* Configure rising/falling edges */

  modifyreg32(STM32L5_EXTI_RTSR1,
              risingedge ? 0 : exti,
              risingedge ? exti : 0);
  modifyreg32(STM32L5_EXTI_FTSR1,
              fallingedge ? 0 : exti,
              fallingedge ? exti : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32L5_EXTI_EMR1,
              event ? 0 : exti,
              event ? exti : 0);
  modifyreg32(STM32L5_EXTI_IMR1,
              func ? 0 : exti,
              func ? exti : 0);

  return OK;
}
