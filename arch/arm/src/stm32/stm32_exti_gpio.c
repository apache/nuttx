/****************************************************************************
 * arch/arm/src/stm32/stm32_exti_gpio.c
 *
 *   Copyright (C) 2009, 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "arm_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"

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

  putreg32(0x0001, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[0].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[0].callback;
      void  *cbarg    = g_gpio_callbacks[0].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti1_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0002, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[1].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[1].callback;
      void  *cbarg    = g_gpio_callbacks[1].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti2_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0004, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[2].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[2].callback;
      void  *cbarg    = g_gpio_callbacks[2].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti3_isr(int irq, void *context, void  * arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0008, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[3].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[3].callback;
      void  *cbarg    = g_gpio_callbacks[3].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int stm32_exti4_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x0010, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[4].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[4].callback;
      void  *cbarg    = g_gpio_callbacks[4].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

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

static int stm32_exti95_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, arg, 5, 9);
}

static int stm32_exti1510_isr(int irq, void *context, void *arg)
{
  return stm32_exti_multiisr(irq, context, arg, 10, 15);
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
  FAR struct gpio_callback_s *shared_cbs;
  uint32_t pin = pinset & GPIO_PIN_MASK;
  uint32_t exti = STM32_EXTI_BIT(pin);
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
