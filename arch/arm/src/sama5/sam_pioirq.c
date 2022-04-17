/****************************************************************************
 * arch/arm/src/sama5/sam_pioirq.c
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
#include "hardware/sam_pio.h"
#include "hardware/sam_pmc.h"

#include "sam_pio.h"
#include "sam_periphclks.h"

#ifdef CONFIG_SAMA5_PIO_IRQ

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
 * Name: sam_piobase
 *
 * Description:
 *   Return the base address of the PIO register set
 *
 ****************************************************************************/

static inline uint32_t sam_piobase(pio_pinset_t pinset)
{
  int port = (pinset & PIO_PORT_MASK) >> PIO_PORT_SHIFT;
  return sam_pion_vbase(port >> PIO_PORT_SHIFT);
}

/****************************************************************************
 * Name: sam_piopin
 *
 * Description:
 *   Return the base address of the PIO register set
 *
 ****************************************************************************/

static inline int sam_piopin(pio_pinset_t pinset)
{
  return 1 << ((pinset & PIO_PIN_MASK) >> PIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: sam_irqbase
 *
 * Description:
 *   Return PIO information associated with this IRQ
 *
 ****************************************************************************/

static int sam_irqbase(int irq, uint32_t *base, int *pin)
{
  if (irq >= SAM_IRQ_NINT)
    {
#ifdef CONFIG_SAMA5_PIOA_IRQ
      if (irq <= SAM_IRQ_PA31)
        {
          *base = SAM_PIOA_VBASE;
          *pin  = irq - SAM_IRQ_PA0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMA5_PIOB_IRQ
      if (irq <= SAM_IRQ_PB31)
        {
          *base = SAM_PIOB_VBASE;
          *pin  = irq - SAM_IRQ_PB0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMA5_PIOC_IRQ
      if (irq <= SAM_IRQ_PC31)
        {
          *base = SAM_PIOC_VBASE;
          *pin  = irq - SAM_IRQ_PC0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMA5_PIOD_IRQ
      if (irq <= SAM_IRQ_PD31)
        {
          *base = SAM_PIOD_VBASE;
          *pin  = irq - SAM_IRQ_PD0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMA5_PIOE_IRQ
      if (irq <= SAM_IRQ_PE31)
        {
          *base = SAM_PIOE_VBASE;
          *pin  = irq - SAM_IRQ_PE0;
          return OK;
        }
#endif

#ifdef CONFIG_SAMA5_PIOF_IRQ
      if (irq <= SAM_IRQ_PF31)
        {
          *base = SAM_PIOF_VBASE;
          *pin  = irq - SAM_IRQ_PF0;
          return OK;
        }
#endif
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sam_pioa/b/c/d/e/finterrupt
 *
 * Description:
 *   Receive PIOA/B/C/D/E/F interrupts
 *
 ****************************************************************************/

static int sam_piointerrupt(uint32_t base, int irq0, void *context)
{
  uint32_t pending;
  uint32_t bit;
  int      irq;

  pending = getreg32(base + SAM_PIO_ISR_OFFSET) & getreg32(base +
          SAM_PIO_IMR_OFFSET);
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

#ifdef CONFIG_SAMA5_PIOA_IRQ
static int sam_pioainterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOA_VBASE, SAM_IRQ_PA0, context);
}
#endif

#ifdef CONFIG_SAMA5_PIOB_IRQ
static int sam_piobinterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOB_VBASE, SAM_IRQ_PB0, context);
}
#endif

#ifdef CONFIG_SAMA5_PIOC_IRQ
static int sam_piocinterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOC_VBASE, SAM_IRQ_PC0, context);
}
#endif

#ifdef CONFIG_SAMA5_PIOD_IRQ
static int sam_piodinterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOD_VBASE, SAM_IRQ_PD0, context);
}
#endif

#ifdef CONFIG_SAMA5_PIOE_IRQ
static int sam_pioeinterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOE_VBASE, SAM_IRQ_PE0, context);
}
#endif

#ifdef CONFIG_SAMA5_PIOF_IRQ
static int sam_piofinterrupt(int irq, void *context, void *arg)
{
  return sam_piointerrupt(SAM_PIOF_VBASE, SAM_IRQ_PF0, context);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   PIO pins.
 *
 ****************************************************************************/

void sam_pioirqinitialize(void)
{
  /* Configure PIOA interrupts */

#ifdef CONFIG_SAMA5_PIOA_IRQ
  /* Enable PIOA clocking */

  sam_pioa_enableclk();

  /* Clear and disable all PIOA interrupts */

  getreg32(SAM_PIOA_ISR);
  putreg32(0xffffffff, SAM_PIOA_IDR);

  /* Attach and enable the PIOA IRQ */

  irq_attach(SAM_IRQ_PIOA, sam_pioainterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOA);
#endif

  /* Configure PIOB interrupts */

#ifdef CONFIG_SAMA5_PIOB_IRQ
  /* Enable PIOB clocking */

  sam_piob_enableclk();

  /* Clear and disable all PIOB interrupts */

  getreg32(SAM_PIOB_ISR);
  putreg32(0xffffffff, SAM_PIOB_IDR);

  /* Attach and enable the PIOB IRQ */

  irq_attach(SAM_IRQ_PIOB, sam_piobinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOB);
#endif

  /* Configure PIOC interrupts */

#ifdef CONFIG_SAMA5_PIOC_IRQ
  /* Enable PIOC clocking */

  sam_pioc_enableclk();

  /* Clear and disable all PIOC interrupts */

  getreg32(SAM_PIOC_ISR);
  putreg32(0xffffffff, SAM_PIOC_IDR);

  /* Attach and enable the PIOC IRQ */

  irq_attach(SAM_IRQ_PIOC, sam_piocinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOC);
#endif

  /* Configure PIOD interrupts */

#ifdef CONFIG_SAMA5_PIOD_IRQ
  /* Enable PIOD clocking */

  sam_piod_enableclk();

  /* Clear and disable all PIOD interrupts */

  getreg32(SAM_PIOD_ISR);
  putreg32(0xffffffff, SAM_PIOD_IDR);

  /* Attach and enable the PIOC IRQ */

  irq_attach(SAM_IRQ_PIOD, sam_piodinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOD);
#endif

  /* Configure PIOE interrupts */

#ifdef CONFIG_SAMA5_PIOE_IRQ
  /* Enable PIOE clocking */

  sam_pioe_enableclk();

  /* Clear and disable all PIOE interrupts */

  getreg32(SAM_PIOE_ISR);
  putreg32(0xffffffff, SAM_PIOE_IDR);

  /* Attach and enable the PIOE IRQ */

  irq_attach(SAM_IRQ_PIOE, sam_pioeinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOE);
#endif

  /* Configure PIOF interrupts */

#ifdef CONFIG_SAMA5_PIOF_IRQ
  /* Enable PIOF clocking */

  sam_piof_enableclk();

  /* Clear and disable all PIOF interrupts */

  getreg32(SAM_PIOF_ISR);
  putreg32(0xffffffff, SAM_PIOF_IDR);

  /* Attach and enable the PIOF IRQ */

  irq_attach(SAM_IRQ_PIOF, sam_piofinterrupt, NULL);
  up_enable_irq(SAM_IRQ_PIOF);
#endif
}

/****************************************************************************
 * Name: sam_pioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ****************************************************************************/

void sam_pioirq(pio_pinset_t pinset)
{
#if defined(SAM_PIO_ISLR_OFFSET)
  uint32_t regval;
#endif
#if defined(SAM_PIO_ISLR_OFFSET) || defined(_PIO_INT_AIM)
  uint32_t base = sam_piobase(pinset);
  int      pin  = sam_piopin(pinset);
#endif

#if defined(SAM_PIO_ISLR_OFFSET)
  /* Enable writing to PIO registers.  The following registers are protected:
   *
   *  - PIO Enable/Disable Registers (PER/PDR)
   *  - PIO Output Enable/Disable Registers (OER/ODR)
   *  - PIO Interrupt Security Level Register (ISLR)
   *  - PIO Input Filter Enable/Disable Registers (IFER/IFDR)
   *  - PIO Multi-driver Enable/Disable Registers (MDER/MDDR)
   *  - PIO Pull-Up Enable/Disable Registers (PUER/PUDR)
   *  - PIO Peripheral ABCD Select Register 1/2 (ABCDSR1/2)
   *  - PIO Output Write Enable/Disable Registers
   *  - PIO Pad Pull-Down Enable/Disable Registers (PPER/PPDR)
   *
   * I suspect that the default state is the WPMR is unprotected, so these
   * operations could probably all be avoided.
   */

  putreg32(PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);

  /* Is the interrupt secure? */

  regval = getreg32(base + SAM_PIO_ISLR_OFFSET);
  if ((pinset & PIO_INT_SECURE) != 0)
    {
      /* Yes.. make sure that the corresponding bit in ISLR is cleared */

      regval &= ~pin;
    }
  else
    {
      /* Yes.. make sure that the corresponding bit in ISLR is set */

      regval |= pin;
    }

  putreg32(regval, base + SAM_PIO_ISLR_OFFSET);
#endif

  /* Are any additional interrupt modes selected? */

#ifdef _PIO_INT_AIM
  if ((pinset & _PIO_INT_AIM) != 0)
    {
      /* Yes.. Enable additional interrupt mode */

      putreg32(pin, base + SAM_PIO_AIMER_OFFSET);

      /* Level or edge detected interrupt? */

      if ((pinset & _PIO_INT_LEVEL) != 0)
        {
          putreg32(pin, base + SAM_PIO_LSR_OFFSET); /* Level */
        }
      else
        {
          putreg32(pin, base + SAM_PIO_ESR_OFFSET); /* Edge */
        }

      /* High level/rising edge or low level /falling edge? */

      if ((pinset & _PIO_INT_RH) != 0)
        {
          /* High level/Rising edge */

          putreg32(pin, base + SAM_PIO_REHLSR_OFFSET);
        }
      else
        {
          /* Low level/Falling edge */

          putreg32(pin, base + SAM_PIO_FELLSR_OFFSET);
        }
    }
  else
    {
      /* No.. Disable additional interrupt mode */

      putreg32(pin, base + SAM_PIO_AIMDR_OFFSET);
    }
#endif

#if defined(SAM_PIO_ISLR_OFFSET)
  /* Disable writing to PIO registers */

  putreg32(PIO_WPMR_WPEN | PIO_WPMR_WPKEY, base + SAM_PIO_WPMR_OFFSET);
#endif
}

/****************************************************************************
 * Name: sam_pioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

void sam_pioirqenable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam_irqbase(irq, &base, &pin) == OK)
    {
      /* Clear (all) pending interrupts and enable this pin interrupt */

      getreg32(base + SAM_PIO_ISR_OFFSET);
      putreg32((1 << pin), base + SAM_PIO_IER_OFFSET);
    }
}

/****************************************************************************
 * Name: sam_pioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

void sam_pioirqdisable(int irq)
{
  uint32_t base;
  int      pin;

  if (sam_irqbase(irq, &base, &pin) == OK)
    {
      /* Disable this pin interrupt */

      putreg32((1 << pin), base + SAM_PIO_IDR_OFFSET);
    }
}

#endif /* CONFIG_SAMA5_PIO_IRQ */
