/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_exti_gpio.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32F0L0G0_HAVE_IP_EXTI_V2)
#  define STM32_EXTI_FTSR             STM32_EXTI_FTSR1
#  define STM32_EXTI_RTSR             STM32_EXTI_RTSR1
#  define STM32_EXTI_IMR              STM32_EXTI_IMR1
#  define STM32_EXTI_EMR              STM32_EXTI_EMR1
#endif

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

#if defined(CONFIG_STM32F0L0G0_HAVE_IP_EXTI_V1)
static int stm32_exti_multiisr(int irq, void *context, void *arg,
                               int first, int last)
{
  uint32_t pr;
  int pin;
  int ret = OK;

  /* Examine the state of each pin in the group */

  pr = getreg32(STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  for (pin = first; pin <= last; pin++)
    {
      /* Is an interrupt pending on this pin? */

      uint32_t mask = (1 << pin);
      if ((pr & mask) != 0)
        {
          /* Clear the pending interrupt */

          putreg32(mask, STM32_EXTI_PR);

          /* And dispatch the interrupt to the handler */

          if (g_gpio_callbacks[pin].callback != NULL)
            {
              xcpt_t callback = g_gpio_callbacks[pin].callback;
              void  *cbarg    = g_gpio_callbacks[pin].arg;
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
#elif defined(CONFIG_STM32F0L0G0_HAVE_IP_EXTI_V2)
static int stm32_exti_multiisr(int irq, void *context, void *arg,
                               int first, int last)
{
  uint32_t rpr;
  uint32_t fpr;
  int pin;
  int ret = OK;

  /* Examine the state of each pin in the group.
   * NOTE: We don't distinguish rising/falling edge!
   */

  rpr = getreg32(STM32_EXTI_RPR1);
  fpr = getreg32(STM32_EXTI_FPR1);

  /* And dispatch the interrupt to the handler */

  for (pin = first; pin <= last; pin++)
    {
      /* Is an interrupt pending on this pin? */

      uint32_t mask = (1 << pin);
      if (((rpr & mask) != 0) || ((fpr & mask) != 0))
        {
          /* Clear the pending interrupt */

          putreg32(mask, STM32_EXTI_RPR1);
          putreg32(mask, STM32_EXTI_FPR1);

          /* And dispatch the interrupt to the handler */

          if (g_gpio_callbacks[pin].callback != NULL)
            {
              xcpt_t callback = g_gpio_callbacks[pin].callback;
              void  *cbarg    = g_gpio_callbacks[pin].arg;
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
#endif

static int stm32_exti01_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, arg, 0, 1);
}

static int stm32_exti23_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, arg, 2, 3);
}

static int stm32_exti415_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, arg, 4, 15);
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
  uint32_t exti = STM32_EXTI_BIT(pin);
  int      irq;
  xcpt_t   handler;
  int      nshared;
  int      i;

  /* Select the interrupt handler for this EXTI pin */

  if (pin < 2)
    {
      irq        = STM32_IRQ_EXTI0_1;
      handler    = stm32_exti01_isr;
      shared_cbs = &g_gpio_callbacks[0];
      nshared    = 2;
    }
  else if (pin < 4)
    {
      irq        = STM32_IRQ_EXTI2_3;
      handler    = stm32_exti23_isr;
      shared_cbs = &g_gpio_callbacks[2];
      nshared    = 2;
    }
  else
    {
      irq        = STM32_IRQ_EXTI4_15;
      handler    = stm32_exti415_isr;
      shared_cbs = &g_gpio_callbacks[4];
      nshared    = 12;
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

  modifyreg32(STM32_EXTI_RTSR,
              risingedge ? 0 : exti,
              risingedge ? exti : 0);
  modifyreg32(STM32_EXTI_FTSR,
              fallingedge ? 0 : exti,
              fallingedge ? exti : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_EMR,
              event ? 0 : exti,
              event ? exti : 0);
  modifyreg32(STM32_EXTI_IMR,
              func ? 0 : exti,
              func ? exti : 0);

  return OK;
}
