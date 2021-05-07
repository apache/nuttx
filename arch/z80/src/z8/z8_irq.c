/****************************************************************************
 * arch/z80/src/z8/z8_irq.c
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

#include <ez8.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip/switch.h"
#include "z80_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This structure holds information about the current interrupt processing
 * state
 */

struct z8_irqstate_s g_z8irqstate;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear and disable all interrupts.  Set all to priority 0. */

  putreg8(0xff, IRQ0);
  putreg8(0xff, IRQ1);
  putreg8(0xff, IRQ2);

  putreg16(0x0000, IRQ0EN);
  putreg16(0x0000, IRQ1EN);
  putreg16(0x0000, IRQ2EN);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  EI();
#endif
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable all interrupts; return previous interrupt state.
 *   REVISIT:  Doesn't TDI() do all of this?
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  /* Bit 7 (IRQE) of the IRQCTL register determines if interrupts were
   * enabled when this function was called.
   */

  register irqstate_t retval = getreg8(IRQCTL);

  /* Disable interrupts */

  DI();

  /* Return the previous interrupt state */

  return retval;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous interrupt state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  /* Bit 7 (IRQE) of the IRQCTL register determines if interrupts were
   * enabled when up_irq_save() was called.
   */

  if ((flags & 0x80) != 0)
    {
      /* The IRQE bit was set, re-enable interrupts.
       * REVISIT: Could not RI() so all of this?
       */

      EI();
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable all interrupts; return previous interrupt state
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  /* Bit 7 (IRQE) of the IRQCTL register determines if interrupts were
   * enabled when this function was called.
   */

  register irqstate_t retval = getreg8(IRQCTL);

  /* Enable interrupts */

  EI();

  /* Return the previous interrupt state */

  return retval;
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  /* System exceptions cannot be disabled */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting interrupt disabled.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8((getreg8(IRQ0ENH) & ~Z8_IRQ0_BIT(irq)), IRQ0ENH);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8((getreg8(IRQ1ENH) & ~Z8_IRQ1_BIT(irq)), IRQ1ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(IRQ2ENH) & ~Z8_IRQ2_BIT(irq)), IRQ2ENH);
        }
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  /* System exceptions cannot be disabled */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Enable the interrupt by setting the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting in "nominal" interrupt
       * priority.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8((getreg8(IRQ0ENH) | Z8_IRQ0_BIT(irq)), IRQ0ENH);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8((getreg8(IRQ1ENH) | Z8_IRQ1_BIT(irq)), IRQ1ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(IRQ2ENH) | Z8_IRQ2_BIT(irq)), IRQ2ENH);
        }
    }
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  /* System exceptions cannot be disabled or acknowledged */

  if (irq >= Z8_IRQ0_MIN)
    {
      /* Acknowledge the interrupt by setting the* corresponding bit in the
       * IRQ status register.
       */

      if (irq <= Z8_IRQ0_MAX)
        {
           putreg8(Z8_IRQ0_BIT(irq), IRQ0);
        }
      else if (irq <= Z8_IRQ1_MAX)
        {
           putreg8(Z8_IRQ1_BIT(irq), IRQ2);
        }
      else if (irq < NR_IRQS)
        {
           putreg8(Z8_IRQ2_BIT(irq), IRQ2);
        }
    }
}
