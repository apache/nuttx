/****************************************************************************
 * arch/arm/src/max326xx/max32690/max32690_gpioirq.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "max326_gpio.h"

extern spinlock_t g_max32690_gpio_lock;

typedef struct
{
    uint32_t int_enabled;

    gpio_irq_function_t irq_handler[32];
}gpio_interrupt_infos;

static gpio_interrupt_infos irq_infos[4];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_gpio0_interrupt
 *
 * Description:
 *   GPIO0 pin interrupt handler.
 *
 ****************************************************************************/

static int max32690_gpio_interrupt(int irq, void *context, void *arg)
{
    uint32_t bank = (uint32_t)arg;

    max32690_gpio_regs_t *gpio = max32690_get_gpio_bank_regptr(bank);

      while (gpio->intfl)
        {
          uint32_t bit_pos = 31 - __builtin_clz(gpio->intfl); /* highest bit set */
          uint32_t mask = 1U << bit_pos;                      /* mask for the set bit */

          if (gpio->intfl & mask)
            {
              gpio->intfl_clr = mask;

              if (irq_infos[bank].irq_handler[bit_pos] != 0)
                {
                   irq_infos[bank].irq_handler[bit_pos](bank, bit_pos);
                }
            }
        }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_gpio_irqconfig
 *
 * Description:
 *   Configure a pin for interrupt operation.
 *
 * Assumptions:
 *
 ****************************************************************************/

void max32690_gpio_irq_enable(max32690_pinconfig_t pinconf)
{
  uint32_t pinmask;
  uint32_t intmode;
  uint32_t intpol;
  uint32_t intdual;

  max32690_gpio_regs_t *gpio;
  gpio = max32690_get_gpio_bank_regptr(pinconf.gpio_bank);

  irqstate_t flags = spin_lock_irqsave(&g_max32690_gpio_lock);

  DEBUGASSERT(pinconf.pin <= GPIO_PINMAX);
  pinmask = 1 << pinconf.pin;

  intmode = 0;
  intpol  = 0;
  intdual = 0;

  switch (pinconf.config & GPIO_INT_MASK)
    {
      default:
        DEBUGPANIC();
        return;

      case GPIO_INTFE:     /* Edge triggered, falling edge */
        intmode = pinmask; /* Edge triggered */
        break;

      case GPIO_INTRE:     /* Edge triggered, rising edge */
        intmode = pinmask; /* Edge triggered */
        intpol  = pinmask; /* Rising edge */
        break;

      case GPIO_INTBOTH:   /* Edge triggered, both edges */
        intmode = pinmask; /* Edge triggered */
        intdual = pinmask; /* Both edges */
        break;

      case GPIO_INTLOW:    /* Level triggered, low level */
        break;

      case GPIO_INTHIGH:   /* Level triggered, high level */
        intpol  = pinmask; /* High level */
        break;
    }

  /* Configure the interrupt modes */

  gpio->intmode &= ~pinmask;
  gpio->intmode |= intmode;

  gpio->intpol &= ~pinmask;
  gpio->intpol |= intpol;

  gpio->dualedge &= ~pinmask;
  gpio->dualedge |= intdual;

  /* enable gpio the interrupt */

  gpio->inten_set |= pinmask;

  int bank = pinconf.gpio_bank;
  irq_infos[bank].irq_handler[pinconf.pin] = pinconf.irq_handler;

  /* enable the interrupt for the gpio bank */

  if (irq_infos[pinconf.gpio_bank].int_enabled == 0)
  {
      switch (pinconf.gpio_bank)
      {
        case 0:
            irq_attach(MAX32690_IRQ_GPIO0, max32690_gpio_interrupt,
                (void *)0);
            up_enable_irq(MAX32690_IRQ_GPIO0);
            break;
        case 1:
            irq_attach(MAX32690_IRQ_GPIO1, max32690_gpio_interrupt,
                (void *)1);
            up_enable_irq(MAX32690_IRQ_GPIO1);
            break;
        case 2:
            irq_attach(MAX32690_IRQ_GPIO2, max32690_gpio_interrupt,
                (void *)2);
            up_enable_irq(MAX32690_IRQ_GPIO2);
            break;
        case 3:
            irq_attach(MAX32690_IRQ_GPIO3, max32690_gpio_interrupt,
                (void *)3);
            up_enable_irq(MAX32690_IRQ_GPIO3);
            break;
        default:
            DEBUGPANIC();
            return;
        }
    }

    spin_unlock_irqrestore(&g_max32690_gpio_lock, flags);
}

/****************************************************************************
 * Name: max326_gpio_irqdisable
 *
 * Description:
 *   Disable a GPIO pin interrupt.
 *   This function should not be called directly but,
 *   rather through up_disable_irq();
 *
 * Assumptions:
 *   We are in a critical section.
 *
 ****************************************************************************/

void max32690_gpio_irq_disable(max32690_pinconfig_t pinconf)
{
      max32690_gpio_regs_t *gpio;
      uint32_t pinmask = 1 << pinconf.pin;

      gpio = max32690_get_gpio_bank_regptr(pinconf.gpio_bank);

      gpio->inten_clr |= pinmask;

      /* critical section */

      irqstate_t flags = spin_lock_irqsave(&g_max32690_gpio_lock);

      irq_infos[pinconf.gpio_bank].irq_handler[pinconf.pin] = 0;

      if (gpio->inten == 0)
        {
          switch (pinconf.gpio_bank)
          {
            case 0:
              irq_detach(MAX32690_IRQ_GPIO0);
              up_disable_irq(MAX32690_IRQ_GPIO0);
              break;
            case 1:
              irq_detach(MAX32690_IRQ_GPIO1);
              up_disable_irq(MAX32690_IRQ_GPIO1);
              break;
            case 2:
              irq_detach(MAX32690_IRQ_GPIO2);
              up_disable_irq(MAX32690_IRQ_GPIO2);
              break;
            case 3:
              irq_detach(MAX32690_IRQ_GPIO3);
              up_disable_irq(MAX32690_IRQ_GPIO3);
              break;
          }
        }

      spin_unlock_irqrestore(&g_max32690_gpio_lock, flags);
}

