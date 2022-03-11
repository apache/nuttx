/****************************************************************************
 * arch/arm/src/stm32h7/stm32_exti_gpio.c
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
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_STM32H7_STM32H7X3XX) || defined(CONFIG_STM32H7_STM32H7X7XX)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  xcpt_t callback;
  void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to each EXTI */

static struct gpio_callback_s g_gpio_callbacks[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Interrupt Service Routines - Dispatchers
 ****************************************************************************/

static int stm32_exti0_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0001, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[0].callback != NULL)
    {
       xcpt_t callback = g_gpio_callbacks[0].callback;
       void   *cbarg   = g_gpio_callbacks[0].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti1_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0002, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[1].callback != NULL)
    {
       xcpt_t callback = g_gpio_callbacks[1].callback;
       void   *cbarg   = g_gpio_callbacks[1].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti2_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0004, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[2].callback != NULL)
    {
       xcpt_t callback = g_gpio_callbacks[2].callback;
       void   *cbarg   = g_gpio_callbacks[2].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti3_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0008, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[3].callback != NULL)
    {
       xcpt_t callback = g_gpio_callbacks[3].callback;
       void   *cbarg   = g_gpio_callbacks[3].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti4_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0010, STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[4].callback != NULL)
    {
       xcpt_t callback = g_gpio_callbacks[4].callback;
       void   *cbarg   = g_gpio_callbacks[4].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti_multiisr(int irq, void *context, int first, int last)
{
  uint32_t pr;
  int pin;
  int ret = OK;

  /* Examine the state of each pin in the group */

  pr = getreg32(STM32_EXTI_CPUPR1);

  /* And dispatch the interrupt to the handler */

  for (pin = first; pin <= last; pin++)
    {
      /* Is an interrupt pending on this pin? */

      uint32_t mask = (1 << pin);
      if ((pr & mask) != 0)
        {
          /* Clear the pending interrupt */

          putreg32(mask, STM32_EXTI_CPUPR1);

          /* And dispatch the interrupt to the handler */

          if (g_gpio_callbacks[pin].callback != NULL)
            {
              xcpt_t callback = g_gpio_callbacks[pin].callback;
              void   *cbarg   = g_gpio_callbacks[pin].arg;
              int tmp;

              tmp = callback(irq, context, cbarg);
              if (tmp < 0)
                {
                  ret = tmp;
                }
            }
        }
    }

  return ret;
}

static int stm32_exti95_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, 5, 9);
}

static int stm32_exti1510_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, 10, 15);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int stm32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{
  struct gpio_callback_s *shared_cbs;
  uint32_t pin = pinset & GPIO_PIN_MASK;
  uint32_t exti = STM32_EXTI_MASK(pin);
  int      irq;
  xcpt_t   handler;
  int      nshared;
  int      i;

  /* Select the interrupt handler for this EXTI pin */

  if (pin < 5)
    {
      irq        = pin + STM32_IRQ_EXTI0;
      nshared    = 1;
      shared_cbs = &g_gpio_callbacks[pin];
      switch (pin)
        {
          case 0:
            handler = stm32_exti0_isr;
            break;

          case 1:
            handler = stm32_exti1_isr;
            break;

          case 2:
            handler = stm32_exti2_isr;
            break;

          case 3:
            handler = stm32_exti3_isr;
            break;

          default:
            handler = stm32_exti4_isr;
            break;
        }
    }
  else if (pin < 10)
    {
      irq        = STM32_IRQ_EXTI95;
      handler    = stm32_exti95_isr;
      shared_cbs = &g_gpio_callbacks[5];
      nshared    = 5;
    }
  else
    {
      irq        = STM32_IRQ_EXTI1510;
      handler    = stm32_exti1510_isr;
      shared_cbs = &g_gpio_callbacks[10];
      nshared    = 6;
    }

  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

  g_gpio_callbacks[pin].callback = func;
  g_gpio_callbacks[pin].arg      = arg;

  /* Install external interrupt handlers */

  if (func)
    {
      irq_attach(irq, handler, NULL);
      up_enable_irq(irq);
    }
  else
    {
      /* Only disable IRQ if shared handler does not have any active
       * callbacks.
       */

      for (i = 0; i < nshared; i++)
        {
          if (shared_cbs[i].callback != NULL)
            {
              break;
            }
        }

      if (i == nshared)
        {
          up_disable_irq(irq);
        }
    }

  /* Configure GPIO, enable EXTI line enabled if event or interrupt is
   * enabled.
   */

  if (event || func)
    {
      pinset |= GPIO_EXTI;
    }

  stm32_configgpio(pinset);

  /* Configure rising/falling edges */

  modifyreg32(STM32_EXTI_RTSR1,
              risingedge ? 0 : exti,
              risingedge ? exti : 0);
  modifyreg32(STM32_EXTI_FTSR1,
              fallingedge ? 0 : exti,
              fallingedge ? exti : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_CPUEMR1,
              event ? 0 : exti,
              event ? exti : 0);
  modifyreg32(STM32_EXTI_CPUIMR1,
              func ? 0 : exti,
              func ? exti : 0);

  return OK;
}

#endif /* CONFIG_STM32H7_STM32H7X3XX || CONFIG_STM32H7_STM32H7X7XX */
