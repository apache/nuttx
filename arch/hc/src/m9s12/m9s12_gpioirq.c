/****************************************************************************
 * arch/hc/src/m9s12/m9s12_gpioirq.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "hc_internal.h"
#include "m9s12.h"
#include "m9s12_pim.h"
#include "m9s12_mebi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_gpioirqinitialize
 *
 * Description:
 *   Map an IRQ number to a port address and a bit number.
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
static int hcs12_mapirq(int irq, uint16_t *regaddr, uint8_t *pin)
{
  if (irq >= HCC12_IRQ_PGFIRST)
    {
      /* Port G: Pins 0-7 */

#ifdef CONFIG_HCS12_PORTG_INTS
      if (irq < HCC12_IRQ_PHFIRST)
        {
          *regaddr = HCS12_PIM_PORTG_IE;
          *pin     = irq - HCC12_IRQ_PGFIRST;
          return OK;
        }
#endif

      /* Port H: Pins 0-6 */

#ifdef CONFIG_HCS12_PORTH_INTS
      if (irq < HCC12_IRQ_PJFIRST)
        {
          *regaddr = HCS12_PIM_PORTH_IE;
          *pin     = irq - HCC12_IRQ_PHFIRST;
          return OK;
        }
#endif

      /* Port J: Pins 0-3 and 6-7 */

#ifdef CONFIG_HCS12_PORTJ_INTS
      if (irq < HCC12_IRQ_PJFIRST)
        {
          uint8_t pjpin = irq - HCC12_IRQ_PJFIRST;
          if (irq >= HCS12_IRQ_PJ6)
            {
              pjpin += 2;
            }

          *regaddr = HCS12_PIM_PORTJ_IE;
          *pin     = pjpin;
          return OK;
        }
#endif
    }

  return -EINVAL;
}
#endif /* CONFIG_HCS12_GPIOIRQ */

/****************************************************************************
 * Name: up_gpioa/b/cinterrupt
 *
 * Description:
 *   Receive GPIOA/B/C interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
static int hcs12_interrupt(uint16_t base,
                           int irq0, uint8_t valid, void *context)
{
  uint8_t pending;
  uint8_t bit;
  int     irq;

  /* Get the set of enabled (unmasked) interrupts pending on this port */

  pending = getreg8(base + HCS12_PIM_IF_OFFSET) &&
            getreg8(base + HCS12_PIM_IE_OFFSET);

  /* Then check each bit in the set of interrupts */

  for (bit = 1, irq = irq0; pending != 0; bit <<= 1)
    {
      /* We may need to skip over some bits in the interrupt register
       * (without incrementing the irq value.
       */

      if ((valid & bit) != 0)
        {
          /* This is a real interrupt bit -- Check if an unmasked interrupt
           * is pending.
           */

          if ((pending & bit) != 0)
            {
              /* Yes.. clear the pending interrupt by writing '1' to the
               * flags registers.
               */

              putreg8(bit, base + HCS12_PIM_IF_OFFSET);

              /* Re-deliver the IRQ
               * (recurses! We got here from irq_dispatch!)
               */

              irq_dispatch(irq, context);

              /* Remove this from the set of pending interrupts */

              pending &= ~bit;
            }

          /* Bump up the IRQ number for the next pass through the loop */

          irq++;
        }
    }

  return OK;
}

#ifdef CONFIG_HCS12_PORTG_INTS
static int hcs12_pginterrupt(int irq, void *context, FAR void *arg)
{
  return hcs12_interrupt(HCS12_PIM_PORTG_BASE, HCS12_IRQ_PG0,
                         HCS12_IRQ_PGSET, context);
}
#endif

#ifdef CONFIG_HCS12_PORTH_INTS
static int hcs12_phinterrupt(int irq, void *context, FAR void *arg)
{
  return hcs12_interrupt(HCS12_PIM_PORTH_BASE, HCS12_IRQ_PH0,
                         HCS12_IRQ_PHSET, context);
}
#endif

#ifdef CONFIG_HCS12_PORTJ_INTS
static int hcs12_pjinterrupt(int irq, void *context, FAR void *arg)
{
  return hcs12_interrupt(HCS12_PIM_PORTJ_BASE, HCS12_IRQ_PJ0,
                         HCS12_IRQ_PJSET, context);
}
#endif
#endif /* CONFIG_HCS12_GPIOIRQ */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void hcs12_gpioirqinitialize(void)
{
  /* Disable all GPIO IRQs -- Ports G, H, and J */

  putreg8(0, HCS12_PIM_PORTG_IE);
  putreg8(0, HCS12_PIM_PORTH_IE);
  putreg8(0, HCS12_PIM_PORTJ_IE);

  /* Attach GPIO IRQ interrupt handlers */

#ifdef CONFIG_HCS12_GPIOIRQ
#  ifdef CONFIG_HCS12_PORTG_INTS
  irq_attach(HCS12_IRQ_VPORTG, hcs12_pginterrupt, NULL);
#  endif
#  ifdef CONFIG_HCS12_PORTH_INTS
  irq_attach(HCS12_IRQ_VPORTH, hcs12_phinterrupt, NULL);
#  endif
#  ifdef CONFIG_HCS12_PORTJ_INTS
  irq_attach(HCS12_IRQ_VPORTJ, hcs12_pjinterrupt, NULL);
#  endif
#endif /* CONFIG_HCS12_GPIOIRQ */
}

/****************************************************************************
 * Name: hcs12_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
void hcs12_gpioirqenable(int irq)
{
  uint16_t regaddr;
  uint8_t  pin;

  if (hcs12_mapirq(irq, &regaddr, &pin) == OK)
    {
       irqstate_t flags  = enter_critical_section();
       uint8_t    regval = getreg8(regaddr);
       regval           |= (1 << pin);
       putreg8(regval, regaddr);
       leave_critical_section(flags);
    }
}
#endif /* CONFIG_HCS12_GPIOIRQ */

/****************************************************************************
 * Name: hcs12_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
void hcs12_gpioirqdisable(int irq)
{
  uint16_t regaddr;
  uint8_t  pin;

  if (hcs12_mapirq(irq, &regaddr, &pin) == OK)
    {
       irqstate_t flags  = enter_critical_section();
       uint8_t    regval = getreg8(regaddr);
       regval           &= ~(1 << pin);
       putreg8(regval, regaddr);
       leave_critical_section(flags);
    }
}
#endif /* CONFIG_HCS12_GPIOIRQ */
