/****************************************************************************
 * arch/arm/src/am335x/am335x_gpioirq.c
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

#include "chip.h"
#include "arm_internal.h"
#include "am335x_gpio.h"

#ifdef CONFIG_AM335X_GPIO_IRQ

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_gpio_risingedge
 *
 * Description:
 *   Set/clear rising edge detection.
 *
 ****************************************************************************/

static void am335x_gpio_risingedge(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr = AM335X_GPIO_RDR(am335x_gpion_vbase(port));
  uint32_t mask = GPIO_PIN(pin);

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_RISING)
    {
      modifyreg32(regaddr, 0, mask);
    }
  else
    {
      modifyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: am335x_gpio_fallingedge
 *
 * Description:
 *   Set/clear falling edge detection.
 *
 ****************************************************************************/

static void am335x_gpio_fallingedge(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr = AM335X_GPIO_FDR(am335x_gpion_vbase(port));
  uint32_t mask = GPIO_PIN(pin);

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_FALLING)
    {
      modifyreg32(regaddr, 0, mask);
    }
  else
    {
      modifyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: am335x_gpio_highlevel
 *
 * Description:
 *   Set/clear high level detection.
 *
 ****************************************************************************/

static void am335x_gpio_highlevel(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr = AM335X_GPIO_LDR1(am335x_gpion_vbase(port));
  uint32_t mask = PIO_PIN(pin);

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_HIGHLEVEL)
    {
      modifyreg32(regaddr, 0, mask);
    }
  else
    {
      modifyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: am335x_gpio_lowlevel
 *
 * Description:
 *   Set/clear low level detection.
 *
 ****************************************************************************/

static void am335x_gpio_lowlevel(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uintptr_t regaddr = AM335X_GPIO_LDR0(am335x_gpion_vbase(port));
  uint32_t mask = GPIO_PIN(pin);

  if ((pinset & GPIO_INT_MASK) == GPIO_INT_LOWLEVEL
    {
      modifiyreg32(regaddr, 0, mask);
    }
  else
    {
      modifiyreg32(regaddr, mask, 0);
    }
}

/****************************************************************************
 * Name: am335x_gpio_interrupt
 *
 * Description:
 *   GPIO0/1/2/3 interrupt handler
 *
 ****************************************************************************/

static int am335x_gpio_interrupt(uint32_t base, int irq0, void *context)
{
  uint32_t isr;
  uint32_t mask;
  int      pin;

  /* Clear all pending interrupts */

  isr = getreg32(AM335X_GPIO_ISR0(base));
  putreg32(isr, AM335X_GPIO_ISR0(base));

  /* Then process each pending GPIO interrupt */

  for (pin = 0; (pin < AM335X_GPIO_NPINS) && (isr != 0); pin++)
    {
      mask = GPIO_PIN(pin);
      if ((isr & mask) != 0)
        {
          /* Remove the pending interrupt bit from the mask */

          isr &= ~mask;

          /* Re-deliver the IRQ (recurses! We got here from irq_dispatch!) */

          irq_dispatch(irq0 + pin, context);
        }
    }

  return OK;
}

#ifdef CONFIG_AM335X_GPIO0_IRQ
static int am335x_gpio0_interrupt(int irq, void *context, void *arg)
{
  return am335x_gpio_interrupt(AM335X_GPIO0_VADDR,
                               AM335X_IRQ_GPIO0P0, context);
}
#endif

#ifdef CONFIG_AM335X_GPIO1_IRQ
static int am335x_gpio1_interrupt(int irq, void *context, void *arg)
{
  return am335x_gpio_interrupt(AM335X_GPIO1_VADDR,
                               AM335X_IRQ_GPIO1P0, context);
}
#endif

#ifdef CONFIG_AM335X_GPIO2_IRQ
static int am335x_gpio2_interrupt(int irq, void *context, void *arg)
{
  return am335x_gpio_interrupt(AM335X_GPIO2_VADDR,
                               AM335X_IRQ_GPIO2P0, context);
}
#endif

#ifdef CONFIG_AM335X_GPIO3_IRQ
static int am335x_gpio3_interrupt(int irq, void *context, void *arg)
{
  return am335x_gpio_interrupt(AM335X_GPIO3_VADDR,
                               AM335X_IRQ_GPIO3P0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void am335x_gpio_irqinitialize(void)
{
  /* Configure GPIO0 interrupts */

#ifdef CONFIG_AM335X_GPIO0_IRQ
  /* Enable GPIO0 clocking */

  /* am335x_gpio0_enableclk(); */

  /* Clear and disable all GPIO0 interrupts */

  putreg32(0xffffffff, AM335X_GPIO_ISCR0(AM335X_GPIO0_VADDR));
  putreg32(0xffffffff, AM335X_GPIO_ISR0(AM335X_GPIO0_VADDR));

  /* Disable all event detections */

  putreg32(0, AM335X_GPIO_LDR0(AM335X_GPIO0_VADDR));
  putreg32(0, AM335X_GPIO_LDR1(AM335X_GPIO0_VADDR));
  putreg32(0, AM335X_GPIO_RDR(AM335X_GPIO0_VADDR));
  putreg32(0, AM335X_GPIO_FDR(AM335X_GPIO0_VADDR));

  /* Attach and enable the GPIO0 IRQ */

  irq_attach(AM335X_IRQ_GPIO0A, am335x_gpio0_interrupt, NULL);
  up_enable_irq(AM335X_IRQ_GPIO0A);
#endif

  /* Configure GPIO1 interrupts */

#ifdef CONFIG_AM335X_GPIO1_IRQ
  /* Enable GPIO1 clocking */

  /* am335x_gpio1_enableclk(); */

  /* Clear and disable all GPIO1 interrupts */

  putreg32(0xffffffff, AM335X_GPIO_ISCR0(AM335X_GPIO1_VADDR));
  putreg32(0xffffffff, AM335X_GPIO_ISR0(AM335X_GPIO1_VADDR));

  /* Disable all event detections */

  putreg32(0, AM335X_GPIO_LDR0(AM335X_GPIO1_VADDR));
  putreg32(0, AM335X_GPIO_LDR1(AM335X_GPIO1_VADDR));
  putreg32(0, AM335X_GPIO_RDR(AM335X_GPIO1_VADDR));
  putreg32(0, AM335X_GPIO_FDR(AM335X_GPIO1_VADDR));

  /* Attach and enable the GPIO1 IRQ */

  irq_attach(AM335X_IRQ_GPIO1A, am335x_gpio1_interrupt, NULL);
  up_enable_irq(AM335X_IRQ_GPIO1A);
#endif

  /* Configure GPIO2 interrupts */

#ifdef CONFIG_AM335X_GPIO2_IRQ
  /* Enable GPIO2 clocking */

  /* am335x_gpio2_enableclk(); */

  /* Clear and disable all GPIO2 interrupts */

  putreg32(0xffffffff, AM335X_GPIO_ISCR0(AM335X_GPIO2_VADDR));
  putreg32(0xffffffff, AM335X_GPIO_ISR0(AM335X_GPIO2_VADDR));

  /* Disable all event detections */

  putreg32(0, AM335X_GPIO_LDR0(AM335X_GPIO2_VADDR));
  putreg32(0, AM335X_GPIO_LDR1(AM335X_GPIO2_VADDR));
  putreg32(0, AM335X_GPIO_RDR(AM335X_GPIO2_VADDR));
  putreg32(0, AM335X_GPIO_FDR(AM335X_GPIO2_VADDR));

  /* Attach and enable the GPIO2 IRQ */

  irq_attach(AM335X_IRQ_GPIO2A, am335x_gpio2_interrupt, NULL);
  up_enable_irq(AM335X_IRQ_GPIO2A);
#endif

  /* Configure GPIO3 interrupts */

#ifdef CONFIG_AM335X_GPIO3_IRQ
  /* Enable GPIO3 clocking */

  /* am335x_gpio3_enableclk(); */

  /* Clear and disable all GPIO3 interrupts */

  putreg32(0xffffffff, AM335X_GPIO_ISCR0(AM335X_GPIO3_VADDR));
  putreg32(0xffffffff, AM335X_GPIO_ISR0(AM335X_GPIO3_VADDR));

  /* Disable all event detections */

  putreg32(0, AM335X_GPIO_LDR0(AM335X_GPIO3_VADDR));
  putreg32(0, AM335X_GPIO_LDR1(AM335X_GPIO3_VADDR));
  putreg32(0, AM335X_GPIO_RDR(AM335X_GPIO3_VADDR));
  putreg32(0, AM335X_GPIO_FDR(AM335X_GPIO3_VADDR));

  /* Attach and enable the GPIO3 IRQ */

  irq_attach(AM335X_IRQ_GPIO3A, am335x_gpio3_interrupt, NULL);
  up_enable_irq(AM335X_IRQ_GPIO3A);
#endif
}

/****************************************************************************
 * Name: am335x_gpioirq
 *
 * Description:
 *   Configure interrupt event detection for the specified GPIO pin.
 *
 ****************************************************************************/

void am335x_gpioirq(gpio_pinset_t pinset)
{
  /* Configure pin detection settings */

  am335x_gpio_risingedge(pinset);
  am335x_gpio_fallingedge(pinset);
  am335x_gpio_highlevel(pinset);
  am335x_gpio_lowlevel(pinset);
}

/****************************************************************************
 * Name: am335x_gpio_irqenable
 *
 * Description:
 *   Enable generation of interrupt from GPIO pin
 *
 ****************************************************************************/

void am335x_gpio_irqenable(gpio_pinset_t pinset);
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Clear and enable GPIO pin interrupt */

  putreg32(GPIO_PIN(pin), AM335X_GPIO_ISR0(am335x_gpion_vbase(port)));
  putreg32(GPIO_PIN(pin), AM335X_GPIO_ISSR0(am335x_gpion_vbase(port)));
}

/****************************************************************************
 * Name: am335x_gpio_irqdisable
 *
 * Description:
 *   Disable generation of interrupt from GPIO pin
 *
 ****************************************************************************/

void am335x_gpio_irqdisable(gpio_pinset_t pinset);
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Disable GPIO pin interrupt */

  putreg32(GPIO_PIN(pin), AM335X_GPIO_ISCR0(am335x_gpion_vbase(port)));
}

#endif /* CONFIG_AM335X_GPIO_IRQ */
