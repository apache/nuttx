/****************************************************************************
 * arch/arm/src/samv7/sam_gpioirq.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_pio.h"

#ifdef CONFIG_SAMV7_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpiobase
 *
 * Description:
 *   Return the base address of the GPIO register set
 *
 ****************************************************************************/

static inline uint32_t sam_gpiobase(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  return SAM_PION_BASE(port >> GPIO_PORT_SHIFT);
}

/****************************************************************************
 * Name: sam_gpiopin
 *
 * Description:
 *   Returun the base address of the GPIO register set
 *
 ****************************************************************************/

static inline int sam_gpiopin(gpio_pinset_t pinset)
{
  return 1 << ((pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_irqbase
 *
 * Description:
 *   Return gpio information associated with this IRQ
 *
 ****************************************************************************/

static int sam_irqbase(int irq, uint32_t *base, int *pin)
{
  if (irq >= SAM_IRQ_NIRQS)
    {
#ifdef CONFIG_SAMV7_GPIOA_IRQ
      if (irq <= SAM_IRQ_PA31)
        {
          *base = SAM_PIOA_BASE;
          *pin  = irq - SAM_IRQ_PA0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMV7_GPIOB_IRQ
      if (irq <= SAM_IRQ_PB31)
        {
          *base = SAM_PIOB_BASE;
          *pin  = irq - SAM_IRQ_PB0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMV7_GPIOC_IRQ
      if (irq <= SAM_IRQ_PC31)
        {
          *base = SAM_PIOC_BASE;
          *pin  = irq - SAM_IRQ_PC0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMV7_GPIOD_IRQ
      if (irq <= SAM_IRQ_PD31)
        {
          *base = SAM_PIOD_BASE;
          *pin  = irq - SAM_IRQ_PD0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMV7_GPIOE_IRQ
      if (irq <= SAM_IRQ_PE31)
        {
          *base = SAM_PIOE_BASE;
          *pin  = irq - SAM_IRQ_PE0;
          return OK;
        }
#endif
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sam_gpioa/b/cinterrupt
 *
 * Description:
 *   Receive GPIOA/B/C interrupts
 *
 ****************************************************************************/

static int sam_gpiointerrupt(uint32_t base, int irq0, void *context)
{
  uint32_t pending;
  uint32_t bit;
  int      irq;

  pending = getreg32(base + SAM_PIO_ISR_OFFSET) &
            getreg32(base + SAM_PIO_IMR_OFFSET);
  for (bit = 1, irq = irq0; pending != 0; bit <<= 1, irq++)
    {
      if ((pending & bit) != 0)
        {
          /* Re-deliver the IRQ (recurses! We got here from irq_dispatch!) */

          irq_dispatch(irq, context);

          /* Remove this from the set of pending interrupts */

          pending &= ~bit;
        }
    }

  return OK;
}

#ifdef CONFIG_SAMV7_GPIOA_IRQ
static int sam_gpioainterrupt(int irq, void *context, void *arg)
{
  return sam_gpiointerrupt(SAM_PIOA_BASE, SAM_IRQ_PA0, context);
}
#endif

#ifdef CONFIG_SAMV7_GPIOB_IRQ
static int sam_gpiobinterrupt(int irq, void *context, void *arg)
{
  return sam_gpiointerrupt(SAM_PIOB_BASE, SAM_IRQ_PB0, context);
}
#endif

#ifdef CONFIG_SAMV7_GPIOC_IRQ
static int sam_gpiocinterrupt(int irq, void *context, void *arg)
{
  return sam_gpiointerrupt(SAM_PIOC_BASE, SAM_IRQ_PC0, context);
}
#endif

#ifdef CONFIG_SAMV7_GPIOD_IRQ
static int sam_gpiodinterrupt(int irq, void *context, void *arg)
{
  return sam_gpiointerrupt(SAM_PIOD_BASE, SAM_IRQ_PD0, context);
}
#endif

#ifdef CONFIG_SAMV7_GPIOE_IRQ
static int sam_gpioeinterrupt(int irq, void *context, void *arg)
{
  return sam_gpiointerrupt(SAM_PIOE_BASE, SAM_IRQ_PE0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void sam_gpioirqinitialize(void)
{
  /* Configure GPIOA interrupts */

#ifdef CONFIG_SAMV7_GPIOA_IRQ
  /* Enable GPIOA clocking */

  sam_pioa_enableclk();

  /* Clear and disable all GPIOA interrupts */

  getreg32(SAM_PIOA_ISR);
  putreg32(0xffffffff, SAM_PIOA_IDR);

  /* Attach and enable the GPIOA IRQ */

  irq_attach(SAM_IRQ_PIOA, sam_gpioainterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOA);
#endif

  /* Configure GPIOB interrupts */

#ifdef CONFIG_SAMV7_GPIOB_IRQ
  /* Enable GPIOB clocking */

  sam_piob_enableclk();

  /* Clear and disable all GPIOB interrupts */

  getreg32(SAM_PIOB_ISR);
  putreg32(0xffffffff, SAM_PIOB_IDR);

  /* Attach and enable the GPIOB IRQ */

  irq_attach(SAM_IRQ_PIOB, sam_gpiobinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOB);
#endif

  /* Configure GPIOC interrupts */

#ifdef CONFIG_SAMV7_GPIOC_IRQ
  /* Enable GPIOC clocking */

  sam_pioc_enableclk();

  /* Clear and disable all GPIOC interrupts */

  getreg32(SAM_PIOC_ISR);
  putreg32(0xffffffff, SAM_PIOC_IDR);

  /* Attach and enable the GPIOC IRQ */

  irq_attach(SAM_IRQ_PIOC, sam_gpiocinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOC);
#endif

  /* Configure GPIOD interrupts */

#ifdef CONFIG_SAMV7_GPIOD_IRQ
  /* Enable GPIOD clocking */

  sam_piod_enableclk();

  /* Clear and disable all GPIOD interrupts */

  getreg32(SAM_PIOD_ISR);
  putreg32(0xffffffff, SAM_PIOD_IDR);

  /* Attach and enable the GPIOC IRQ */

  irq_attach(SAM_IRQ_PIOD, sam_gpiodinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOD);
#endif

  /* Configure GPIOE interrupts */

#ifdef CONFIG_SAMV7_GPIOE_IRQ
  /* Enable GPIOE clocking */

  sam_pioe_enableclk();

  /* Clear and disable all GPIOE interrupts */

  getreg32(SAM_PIOE_ISR);
  putreg32(0xffffffff, SAM_PIOE_IDR);

  /* Attach and enable the GPIOE IRQ */

  irq_attach(SAM_IRQ_PIOE, sam_gpioeinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOE);
#endif
}

/****************************************************************************
 * Name: sam_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

void sam_gpioirq(gpio_pinset_t pinset)
{
  uint32_t base = sam_gpiobase(pinset);
  int      pin  = sam_gpiopin(pinset);

  /* Are any additional interrupt modes selected? */

  if ((pinset & _GIO_INT_AIM) != 0)
    {
      /* Yes.. Enable additional interrupt mode */

      putreg32(pin, base + SAM_PIO_AIMER_OFFSET);

      /* Level or edge detected interrupt? */

      if ((pinset & _GPIO_INT_LEVEL) != 0)
        {
          putreg32(pin, base + SAM_PIO_LSR_OFFSET); /* Level */
        }
      else
        {
          putreg32(pin, base + SAM_PIO_ESR_OFFSET); /* Edge */
        }

      /* High level/rising edge or low level /falling edge? */

      if ((pinset & _GPIO_INT_RH) != 0)
        {
          putreg32(pin, base + SAM_PIO_REHLSR_OFFSET); /* High level/Rising edge */
        }
      else
        {
          putreg32(pin, base + SAM_PIO_FELLSR_OFFSET); /* Low level/Falling edge */
        }
    }
  else
    {
      /* No.. Disable additional interrupt mode */

      putreg32(pin, base + SAM_PIO_AIMDR_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void sam_gpioirqenable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam_irqbase(irq, &base, &pin) == OK)
    {
      /* Clear (all) pending interrupts and enable this pin interrupt */

      /* getreg32(base + SAM_PIO_ISR_OFFSET); */

      putreg32((1 << pin), base + SAM_PIO_IER_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

void sam_gpioirqdisable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam_irqbase(irq, &base, &pin) == OK)
    {
      /* Disable this pin interrupt */

      putreg32((1 << pin), base + SAM_PIO_IDR_OFFSET);
    }
}

#endif /* CONFIG_SAMV7_GPIO_IRQ */
