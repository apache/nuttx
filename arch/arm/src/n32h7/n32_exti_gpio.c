/****************************************************************************
 * arch/arm/src/n32h7/n32_exti_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "n32_gpio.h"
#include "n32_exti.h"

/* Content of this file requires verification before it is used with other
 * families
 */

#if defined(CONFIG_N32H7_N32H76X)
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
  uint32_t port;
  uint32_t pin;
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

static int n32_exti0_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x00000001, N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[0].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[0].callback;
      void   *cbarg   = g_gpio_callbacks[0].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int n32_exti1_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x00000002, N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[1].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[1].callback;
      void   *cbarg   = g_gpio_callbacks[1].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int n32_exti2_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x00000004, N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[2].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[2].callback;
      void   *cbarg   = g_gpio_callbacks[2].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int n32_exti3_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x00000008, N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[3].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[3].callback;
      void   *cbarg   = g_gpio_callbacks[3].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int n32_exti4_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  putreg32(0x00000010, N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[4].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[4].callback;
      void   *cbarg   = g_gpio_callbacks[4].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int n32_exti_multiisr(int irq, void *context, int first, int last)
{
  uint32_t pr;
  int pin;
  int ret = OK;

  /* Examine the state of each pin in the group */

  pr = getreg32(N32_EXTI_M7PEND0);

  /* And dispatch the interrupt to the handler */

  for (pin = first; pin <= last; pin++)
    {
      /* Is an interrupt pending on this pin? */

      uint32_t mask = (1 << pin);

      if ((pr & mask) != 0)
        {
          /* Clear the pending interrupt */

          putreg32(mask, N32_EXTI_M7PEND0);

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

static int n32_exti95_isr(int irq, void *context, void *arg)
{
  return n32_exti_multiisr(irq, context, 5, 9);
}

static int n32_exti1510_isr(int irq, void *context, void *arg)
{
  return n32_exti_multiisr(irq, context, 10, 15);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_gpiosetevent
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

int n32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{
  struct gpio_callback_s *shared_cbs;
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t exti;
  uint32_t regval;
  xcpt_t   handler;
  int      nshared;
  int      i;
  int      irq;

  /* Find the first available EXTI line */

  for (exti = 0; exti < 16; exti++)
    {
      if (func && g_gpio_callbacks[exti].callback != NULL &&
          g_gpio_callbacks[exti].port == port &&
          g_gpio_callbacks[exti].pin == pin)
        {
          break;
        }
      else if (func && g_gpio_callbacks[exti].callback == NULL)
        {
          break;
        }
      else if (!func && g_gpio_callbacks[exti].port == port &&
               g_gpio_callbacks[exti].pin == pin)
        {
          break;
        }
    }

  /* Select the interrupt handler for this EXTI line */

  if (exti < 5)
    {
      irq        = exti + N32_IRQ_EXTI0;
      nshared    = 1;
      shared_cbs = &g_gpio_callbacks[exti];
      switch (exti)
        {
          case 0:
            handler = n32_exti0_isr;
            break;

          case 1:
            handler = n32_exti1_isr;
            break;

          case 2:
            handler = n32_exti2_isr;
            break;

          case 3:
            handler = n32_exti3_isr;
            break;

          default:
            handler = n32_exti4_isr;
            break;
        }
    }
  else if (exti < 10)
    {
      irq        = N32_IRQ_EXTI9_5;
      handler    = n32_exti95_isr;
      shared_cbs = &g_gpio_callbacks[5];
      nshared    = 5;
    }
  else if (exti < 16)
    {
      irq        = N32_IRQ_EXTI15_10;
      handler    = n32_exti1510_isr;
      shared_cbs = &g_gpio_callbacks[10];
      nshared    = 6;
    }
  else
    {
      if (func)
        {
          return -EINVAL;
        }
      else
        {
          return OK;
        }
    }

  /* Install external interrupt handlers */

  if (func)
    {
      /* Save the new IRQ handler. */

      g_gpio_callbacks[exti].port = port;
      g_gpio_callbacks[exti].pin = pin;

      regval = getreg32(N32_AFIO_EXTI_CFG(exti >> 2));
      regval &= ~N32_AFIO_EXTICR_EXTI_MASK(exti);
      regval |= (pin * 11 + port) << N32_AFIO_EXTICR_EXTI_SHIFT(exti);
      putreg32(regval, N32_AFIO_EXTI_CFG(exti >> 2));

      g_gpio_callbacks[exti].callback = func;
      g_gpio_callbacks[exti].arg      = arg;

      irq_attach(irq, handler, NULL);
      up_enable_irq(irq);
    }
  else
    {
      /* Clear the previous IRQ handler. */

      g_gpio_callbacks[exti].port = 0;
      g_gpio_callbacks[exti].pin = 0;

      regval = getreg32(N32_AFIO_EXTI_CFG(exti >> 2));
      regval &= ~N32_AFIO_EXTICR_EXTI_MASK(exti);
      putreg32(regval, N32_AFIO_EXTI_CFG(exti >> 2));

      g_gpio_callbacks[exti].callback = NULL;
      g_gpio_callbacks[exti].arg      = NULL;

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

  n32_configgpio(pinset);

  /* Configure rising/falling edges */

  modifyreg32(N32_EXTI_RT_CFG0,
              risingedge ? 0 : (1U << exti),
              risingedge ? (1U << exti) : 0);
  modifyreg32(N32_EXTI_FT_CFG0,
              fallingedge ? 0 : (1U << exti),
              fallingedge ? (1U << exti) : 0);

  /* Enable Events and Interrupts */

  modifyreg32(N32_EXTI_M7EMASK0,
              event ? 0 : (1U << exti),
              event ? (1U << exti) : 0);
  modifyreg32(N32_EXTI_M7IMASK0,
              func ? 0 : (1U << exti),
              func ? (1U << exti) : 0);

  return OK;
}

#endif /* CONFIG_N32H7_N32H76X */
