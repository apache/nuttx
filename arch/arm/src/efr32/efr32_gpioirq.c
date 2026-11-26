/****************************************************************************
 * arch/arm/src/efr32/efr32_gpioirq.c
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/efr32_memorymap.h"
#include "efr32_gpio.h"

#ifdef CONFIG_EFR32_GPIO_IRQ

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_gpio_interrupt
 *
 * Description:
 *   Dispatch the pending, enabled pin interrupts in the given set of
 *   EXTI lines (the even or the odd ones).
 *
 ****************************************************************************/

static int efr32_gpio_interrupt(uint32_t mask, void *context)
{
  uint32_t pending;
  int line;

  pending = getreg32(EFR32_GPIO_IF) & getreg32(EFR32_GPIO_IEN) & mask;
  putreg32(pending, EFR32_GPIO_IF + EFR32_CLR_OFFSET);

  for (line = 0; pending != 0; line++)
    {
      if ((pending & (1 << line)) != 0)
        {
          irq_dispatch(EFR32_IRQ_EXTI(line), context);
          pending &= ~(1 << line);
        }
    }

  return OK;
}

static int efr32_gpio_even_interrupt(int irq, void *context, void *arg)
{
  return efr32_gpio_interrupt(GPIO_EXTI_EVEN, context);
}

static int efr32_gpio_odd_interrupt(int irq, void *context, void *arg)
{
  return efr32_gpio_interrupt(GPIO_EXTI_ODD, context);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_gpioirqinitialize
 *
 * Description:
 *   Disable all pin interrupts and attach the even and odd GPIO interrupt
 *   handlers.  Called from up_irqinitialize().
 *
 ****************************************************************************/

void efr32_gpioirqinitialize(void)
{
  putreg32(0, EFR32_GPIO_IEN);
  putreg32(0, EFR32_GPIO_EXTIRISE);
  putreg32(0, EFR32_GPIO_EXTIFALL);
  putreg32(GPIO_EXTI_ALL, EFR32_GPIO_IF + EFR32_CLR_OFFSET);

  irq_attach(EFR32_IRQ_GPIO_EVEN, efr32_gpio_even_interrupt, NULL);
  irq_attach(EFR32_IRQ_GPIO_ODD, efr32_gpio_odd_interrupt, NULL);

  up_enable_irq(EFR32_IRQ_GPIO_EVEN);
  up_enable_irq(EFR32_IRQ_GPIO_ODD);
}

/****************************************************************************
 * Name: efr32_gpioirq
 *
 * Description:
 *   Select the edges that set the interrupt flag of a pin.  Pin n uses
 *   EXTIn, so only pins 0-11 can interrupt, and only one port per pin
 *   number at a time.  The interrupt is left disabled; attach a handler
 *   to EFR32_IRQ_EXTI(pin) and enable it with up_enable_irq().
 *
 * Returned Value:
 *   OK on success; -EINVAL if the pin has no interrupt line.
 *
 ****************************************************************************/

int efr32_gpioirq(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uintptr_t portsel;
  uintptr_t pinsel;
  uint32_t bit;
  int port;
  int pin;

  port = GPIO_PORTNO(pinset);
  pin  = GPIO_PINNO(pinset);

  if (pin >= GPIO_NEXTI)
    {
      return -EINVAL;
    }

  bit = 1 << pin;

  if (pin < 8)
    {
      portsel = EFR32_GPIO_EXTIPSELL;
      pinsel  = EFR32_GPIO_EXTIPINSELL;
    }
  else
    {
      portsel = EFR32_GPIO_EXTIPSELH;
      pinsel  = EFR32_GPIO_EXTIPINSELH;
    }

  flags = enter_critical_section();

  /* Disable the line while it is changed */

  putreg32(bit, EFR32_GPIO_IEN + EFR32_CLR_OFFSET);

  /* EXTIn takes the port and the pin offset from 4 * (n / 4) */

  modifyreg32(portsel, GPIO_EXTI_MASK(pin),
              (uint32_t)port << GPIO_EXTI_SHIFT(pin));
  modifyreg32(pinsel, GPIO_EXTI_MASK(pin),
              (uint32_t)(pin & 3) << GPIO_EXTI_SHIFT(pin));

  if ((pinset & GPIO_INT_RISING) != 0)
    {
      putreg32(bit, EFR32_GPIO_EXTIRISE + EFR32_SET_OFFSET);
    }
  else
    {
      putreg32(bit, EFR32_GPIO_EXTIRISE + EFR32_CLR_OFFSET);
    }

  if ((pinset & GPIO_INT_FALLING) != 0)
    {
      putreg32(bit, EFR32_GPIO_EXTIFALL + EFR32_SET_OFFSET);
    }
  else
    {
      putreg32(bit, EFR32_GPIO_EXTIFALL + EFR32_CLR_OFFSET);
    }

  /* Drop any flag set by the old configuration */

  putreg32(bit, EFR32_GPIO_IF + EFR32_CLR_OFFSET);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: efr32_gpioirqenable
 *
 * Description:
 *   Enable one pin interrupt (EFR32_IRQ_EXTI0-11).
 *
 ****************************************************************************/

void efr32_gpioirqenable(int irq)
{
  int line = irq - EFR32_IRQ_EXTI0;

  DEBUGASSERT(line >= 0 && line < GPIO_NEXTI);
  putreg32(1 << line, EFR32_GPIO_IEN + EFR32_SET_OFFSET);
}

/****************************************************************************
 * Name: efr32_gpioirqdisable
 *
 * Description:
 *   Disable one pin interrupt (EFR32_IRQ_EXTI0-11).
 *
 ****************************************************************************/

void efr32_gpioirqdisable(int irq)
{
  int line = irq - EFR32_IRQ_EXTI0;

  DEBUGASSERT(line >= 0 && line < GPIO_NEXTI);
  putreg32(1 << line, EFR32_GPIO_IEN + EFR32_CLR_OFFSET);
}

#endif /* CONFIG_EFR32_GPIO_IRQ */
