/****************************************************************************
 * arch/arm/src/sam34/sam_gpioirq.c
 *
 *   Copyright (C) 2010, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "chip/sam_pmc.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  include "chip/sam3u_pio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4E)
#  include "chip/sam4e_pio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  include "chip/sam4s_pio.h"
#else
#  error Unrecognized SAM architecture
#endif

#ifdef CONFIG_SAM34_GPIO_IRQ

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
#ifdef CONFIG_SAM34_GPIOA_IRQ
      if (irq <= SAM_IRQ_PA31)
        {
          *base = SAM_PIOA_BASE;
          *pin  = irq - SAM_IRQ_PA0;
          return OK;
        }
#endif
#ifdef CONFIG_SAM34_GPIOB_IRQ
      if (irq <= SAM_IRQ_PB31)
        {
          *base = SAM_PIOB_BASE;
          *pin  = irq - SAM_IRQ_PB0;
          return OK;
        }
#endif
#ifdef CONFIG_SAM34_GPIOC_IRQ
      if (irq <= SAM_IRQ_PC31)
        {
          *base = SAM_PIOC_BASE;
          *pin  = irq - SAM_IRQ_PC0;
          return OK;
        }
#endif
#ifdef CONFIG_SAM34_GPIOD_IRQ
      if (irq <= SAM_IRQ_PD31)
        {
          *base = SAM_PIOD_BASE;
          *pin  = irq - SAM_IRQ_PD0;
          return OK;
        }
#endif
#ifdef CONFIG_SAM34_GPIOE_IRQ
      if (irq <= SAM_IRQ_PE31)
        {
          *base = SAM_PIOE_BASE;
          *pin  = irq - SAM_IRQ_PE0;
          return OK;
        }
#endif
#ifdef CONFIG_SAM34_GPIOF_IRQ
      if (irq <= SAM_IRQ_PF31)
        {
          *base = SAM_PIOF_BASE;
          *pin  = irq - SAM_IRQ_PF0;
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

  pending = getreg32(base + SAM_PIO_ISR_OFFSET) & getreg32(base + SAM_PIO_IMR_OFFSET);
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

#ifdef CONFIG_SAM34_GPIOA_IRQ
static int sam_gpioainterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOA_BASE, SAM_IRQ_PA0, context);
}
#endif

#ifdef CONFIG_SAM34_GPIOB_IRQ
static int sam_gpiobinterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOB_BASE, SAM_IRQ_PB0, context);
}
#endif

#ifdef CONFIG_SAM34_GPIOC_IRQ
static int sam_gpiocinterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOC_BASE, SAM_IRQ_PC0, context);
}
#endif

#ifdef CONFIG_SAM34_GPIOD_IRQ
static int sam_gpiodinterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOD_BASE, SAM_IRQ_PD0, context);
}
#endif

#ifdef CONFIG_SAM34_GPIOE_IRQ
static int sam_gpioeinterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOE_BASE, SAM_IRQ_PE0, context);
}
#endif

#ifdef CONFIG_SAM34_GPIOF_IRQ
static int sam_gpiofinterrupt(int irq, void *context, FAR void *arg)
{
  return sam_gpiointerrupt(SAM_PIOF_BASE, SAM_IRQ_PF0, context);
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

#ifdef CONFIG_SAM34_GPIOA_IRQ
  /* Enable GPIOA clocking */

  sam_pioa_enableclk();

  /* Clear and disable all GPIOA interrupts */

  (void)getreg32(SAM_PIOA_ISR);
  putreg32(0xffffffff, SAM_PIOA_IDR);

  /* Attach and enable the GPIOA IRQ */

  (void)irq_attach(SAM_IRQ_PIOA, sam_gpioainterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOA);
#endif

  /* Configure GPIOB interrupts */

#ifdef CONFIG_SAM34_GPIOB_IRQ
  /* Enable GPIOB clocking */

  sam_piob_enableclk();

  /* Clear and disable all GPIOB interrupts */

  (void)getreg32(SAM_PIOB_ISR);
  putreg32(0xffffffff, SAM_PIOB_IDR);

  /* Attach and enable the GPIOB IRQ */

  (void)irq_attach(SAM_IRQ_PIOB, sam_gpiobinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOB);
#endif

  /* Configure GPIOC interrupts */

#ifdef CONFIG_SAM34_GPIOC_IRQ
  /* Enable GPIOC clocking */

  sam_pioc_enableclk();

  /* Clear and disable all GPIOC interrupts */

  (void)getreg32(SAM_PIOC_ISR);
  putreg32(0xffffffff, SAM_PIOC_IDR);

  /* Attach and enable the GPIOC IRQ */

  (void)irq_attach(SAM_IRQ_PIOC, sam_gpiocinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOC);
#endif

  /* Configure GPIOD interrupts */

#ifdef CONFIG_SAM34_GPIOD_IRQ
  /* Enable GPIOD clocking */

  sam_piod_enableclk();

  /* Clear and disable all GPIOD interrupts */

  (void)getreg32(SAM_PIOD_ISR);
  putreg32(0xffffffff, SAM_PIOD_IDR);

  /* Attach and enable the GPIOC IRQ */

  (void)irq_attach(SAM_IRQ_PIOD, sam_gpiodinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOD);
#endif

  /* Configure GPIOE interrupts */

#ifdef CONFIG_SAM34_GPIOE_IRQ
  /* Enable GPIOE clocking */

  sam_pioe_enableclk();

  /* Clear and disable all GPIOE interrupts */

  (void)getreg32(SAM_PIOE_ISR);
  putreg32(0xffffffff, SAM_PIOE_IDR);

  /* Attach and enable the GPIOE IRQ */

  (void)irq_attach(SAM_IRQ_PIOE, sam_gpioeinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOE);
#endif

  /* Configure GPIOF interrupts */

#ifdef CONFIG_SAM34_GPIOF_IRQ
  /* Enable GPIOF clocking */

  sam_piof_enableclk();

  /* Clear and disable all GPIOF interrupts */

  (void)getreg32(SAM_PIOF_ISR);
  putreg32(0xffffffff, SAM_PIOF_IDR);

  /* Attach and enable the GPIOF IRQ */

  (void)irq_attach(SAM_IRQ_PIOF, sam_gpiofinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOF);
#endif
}

/************************************************************************************
 * Name: sam_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: sam_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

void sam_gpioirqenable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam_irqbase(irq, &base, &pin) == OK)
    {
      /* Clear (all) pending interrupts and enable this pin interrupt */

      //(void)getreg32(base + SAM_PIO_ISR_OFFSET);
      putreg32((1 << pin), base + SAM_PIO_IER_OFFSET);
    }
}

/************************************************************************************
 * Name: sam_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

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

#endif /* CONFIG_SAM34_GPIO_IRQ */
