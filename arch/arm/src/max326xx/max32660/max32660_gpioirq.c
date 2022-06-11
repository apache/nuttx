/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_gpioirq.c
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

#include "arm_internal.h"
#include "max326_gpio.h"

#ifdef CONFIG_MAX326XX_GPIOIRQ

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

static int max326_gpio0_interrupt(int irq, void *context, void *arg)
{
  uint32_t pending;
  int i;

  /* Get the pending interrupt set */

  pending = getreg32(MAX326_GPIO0_INTFL) & getreg32(MAX326_GPIO0_INTEN) &
            MAX326_GPIO0_ALLPINS;

  for (i = 0; pending != 0 && i < MAX326_GPIO0_NPINS; i++)
    {
      uint32_t pinmask = GPIO_INTEN(i);
      if ((pending & pinmask) != 0)
        {
          /* Dispatch the GPIO interrupt */

          pending &= ~pinmask;
          irq_dispatch(MAX326_IRQ_GPIO1ST + i, context);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max326_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is
 *   called by the OS initialization logic and is not a user interface.
 *
 * Assumptions:
 *   Called early in the boot-up sequence
 *
 ****************************************************************************/

void max326_gpio_irqinitialize(void)
{
  /* Attach the GPIO0 interrupt handler and enable interrupts */

  DEBUGVERIFY(irq_attach(MAX326_IRQ_GPIO0, max326_gpio0_interrupt, NULL));
  up_enable_irq(MAX326_IRQ_GPIO0);
}

/****************************************************************************
 * Name: max326_gpio_irqconfig
 *
 * Description:
 *   Configure a pin for interrupt operation.  This function should not be
 *   called directory but, rather, indirectly through max326_gpio_config().
 *
 * Assumptions:
 *   - The pin interrupt has been disabled and all interrupt related bits
 *     have been set to zero by max436_gpio_config().
 *   - We are called in a critical section.
 *
 ****************************************************************************/

void max326_gpio_irqconfig(max326_pinset_t cfgset)
{
  unsigned int pin;
  uint32_t pinmask;
  uint32_t intmode;
  uint32_t intpol;
  uint32_t intdual;
  uint32_t regval;

  pin     = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  DEBUGASSERT(pin <= GPIO_PINMAX);
  pinmask = 1 << pin;

  intmode = 0;
  intpol  = 0;
  intdual = 0;

  switch (cfgset & GPIO_FUNC_MASK)
    {
      default:
      case GPIO_INPUT:
      case GPIO_OUTPUT:
      case GPIO_ALT1:
      case GPIO_ALT2:
      case GPIO_ALT3:
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

  /* Configure the interrupt */

  regval  = getreg32(MAX326_GPIO0_INTMODE);
  regval &= ~pinmask;
  regval |= intmode;
  putreg32(regval, MAX326_GPIO0_INTMODE);

  regval  = getreg32(MAX326_GPIO0_INTPOL);
  regval &= ~pinmask;
  regval |= intpol;
  putreg32(regval, MAX326_GPIO0_INTPOL);

  regval  = getreg32(MAX326_GPIO0_INTDUALEDGE);
  regval &= ~pinmask;
  regval |= intdual;
  putreg32(regval, MAX326_GPIO0_INTDUALEDGE);
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

void max326_gpio_irqdisable(int irq)
{
  unsigned int pin;

  DEBUGASSERT(irq >= MAX326_IRQ_GPIO1ST && irq <= MAX326_IRQ_GPIOLAST)

  if (irq >= MAX326_IRQ_GPIO1ST && irq <= MAX326_IRQ_GPIOLAST)
    {
      pin = irq - MAX326_IRQ_GPIO1ST;

      /* Modification of registers must be atomic */

      modifyreg32(MAX326_GPIO0_INTEN, 1 << pin, 0);
    }
}

/****************************************************************************
 * Name: max326_gpio_irqenable
 *
 * Description:
 *   Enable a GPIO pin interrupt.
 *   This function should not be called directly but,
 *   rather through up_enable_irq();
 *
 * Assumptions:
 *   We are in a critical section.
 *
 ****************************************************************************/

void max326_gpio_irqenable(int irq)
{
  unsigned int pin;

  DEBUGASSERT(irq >= MAX326_IRQ_GPIO1ST && irq <= MAX326_IRQ_GPIOLAST)

  if (irq >= MAX326_IRQ_GPIO1ST && irq <= MAX326_IRQ_GPIOLAST)
    {
      pin = irq - MAX326_IRQ_GPIO1ST;

      /* Modification of registers must be atomic */

      modifyreg32(MAX326_GPIO0_INTEN, 0, 1 << pin);
    }
}

#endif /* CONFIG_MAX326XX_GPIOIRQ */
