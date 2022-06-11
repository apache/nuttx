/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_exti_gpio.c
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

#include "up_internal.h"
#include "chip.h"
#include "bm3803.h"
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

static struct gpio_callback_s g_gpio_callbacks[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Interrupt Service Routines - Dispatchers
 ****************************************************************************/

static int bm3803_exti0_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* Clear the pending interrupt */

  up_clrpend_irq(BM3803_IRQ_EXTERNAL_0);

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[0].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[0].callback;
      void  *cbarg    = g_gpio_callbacks[0].arg;

      ret = callback(irq, context, cbarg);
    }

  return ret;
}

static int bm3803_exti1_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[1].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[1].callback;
      void  *cbarg    = g_gpio_callbacks[1].arg;

      ret = callback(irq, context, cbarg);
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(BM3803_IRQ_EXTERNAL_1);

  return ret;
}

static int bm3803_exti2_isr(int irq, void *context, void *arg)
{
  int ret = OK;

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[2].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[2].callback;
      void  *cbarg    = g_gpio_callbacks[2].arg;

      ret = callback(irq, context, cbarg);
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(BM3803_IRQ_EXTERNAL_2);

  return ret;
}

static int bm3803_exti3_isr(int irq, void *context, void  * arg)
{
  int ret = OK;

  /* And dispatch the interrupt to the handler */

  if (g_gpio_callbacks[3].callback != NULL)
    {
      xcpt_t callback = g_gpio_callbacks[3].callback;
      void  *cbarg    = g_gpio_callbacks[3].arg;

      ret = callback(irq, context, cbarg);
    }

  /* Clear the pending interrupt */

  up_clrpend_irq(BM3803_IRQ_EXTERNAL_3);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_gpioset_irq
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:     GPIO pin configuration
 *  - enable:     Enables interrupt or not
 *  - trig:       Trig interrupt on level or falling edges
 *  - edge:       Trig interrupt on falling edges or rising edges
 *  - func:       When non-NULL, generate interrupt
 *  - arg:        Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int bm3803_gpioset_irq(uint32_t pinset, bool enable, bool trig, bool edge,
                       xcpt_t func, void *arg)
{
  struct gpio_callback_s *shared_cbs;
  uint32_t pin = pinset & 0x3;

  uint32_t en = 0x80 << (pin * 8);
  uint32_t le = 0x40 << (pin * 8);
  uint32_t pl = 0x20 << (pin * 8);
  uint32_t isel = (pin + 4) << (pin * 8);

  int      irq;
  xcpt_t   handler;
  int      nshared;
  int      i;

  /* Select the interrupt handler for this EXTI pin */

  if (pin < 4)
    {
      irq        = pin + BM3803_IRQ_EXTERNAL_0;
      nshared    = 1;
      shared_cbs = &g_gpio_callbacks[pin];
      switch (pin)
        {
          case 0:
            handler = bm3803_exti0_isr;
            break;

          case 1:
            handler = bm3803_exti1_isr;
            break;

          case 2:
            handler = bm3803_exti2_isr;
            break;

          case 3:
            handler = bm3803_exti3_isr;
            break;

          default:
            handler = bm3803_exti3_isr;
            break;
        }
    }
  else
    {
      return ERROR;
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
       * callbacks
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

  /* Configure GPIO, enable EXTI line enabled if event or interrupt
   * is enabled
   */

  BM3803_REG.pio_dir = BM3803_REG.pio_dir & ~(1 << (pin + 4));

  /* Configure rising/falling edges */

  modifyreg32((uint32_t)&BM3803_REG.pio_irq, le, trig ? le : 0);
  modifyreg32((uint32_t)&BM3803_REG.pio_irq, pl, edge ? pl : 0);
  modifyreg32((uint32_t)&BM3803_REG.pio_irq, 0x1f << (pin * 8), isel);
  modifyreg32((uint32_t)&BM3803_REG.pio_irq, en, enable ? en : 0);
  return OK;
}
