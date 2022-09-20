/****************************************************************************
 * arch/z16/src/z16f/z16f_irq.c
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

#include <sys/types.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "chip.h"
#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Clear and disable all interrupts.  Set all to priority 0. */

  putreg8(0xff, Z16F_IRQ0);
  putreg8(0xff, Z16F_IRQ1);
  putreg8(0xff, Z16F_IRQ2);

  putreg16(0x0000, Z16F_IRQ0_EN);
  putreg16(0x0000, Z16F_IRQ1_EN);
  putreg16(0x0000, Z16F_IRQ2_EN);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  EI();
#endif
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

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Disable the interrupt by clearing the corresponding bit in the
       * appropriate IRQ enable high register.  The enable low
       * register is assumed to be zero, resulting interrupt disabled.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8((getreg8(Z16F_IRQ0_ENH) & ~Z16F_IRQ0_BIT(irq)),
                   Z16F_IRQ0_ENH);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8((getreg8(Z16F_IRQ1_ENH) & ~Z16F_IRQ1_BIT(irq)),
                   Z16F_IRQ1_ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(Z16F_IRQ2_ENH) & ~Z16F_IRQ2_BIT(irq)),
                   Z16F_IRQ2_ENH);
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

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Enable the interrupt by setting the corresponding bit in the
       * appropriate IRQ enable register.  The enable low
       * register is assumed to be zero, resulting in "nominal" interrupt
       * priority.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8((getreg8(Z16F_IRQ0_ENH) | Z16F_IRQ0_BIT(irq)),
                   Z16F_IRQ0_ENH);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8((getreg8(Z16F_IRQ1_ENH) | Z16F_IRQ1_BIT(irq)),
                   Z16F_IRQ1_ENH);
        }
      else if (irq < NR_IRQS)
        {
           putreg8((getreg8(Z16F_IRQ2_ENH) | Z16F_IRQ2_BIT(irq)),
                   Z16F_IRQ2_ENH);
        }
    }
}

/****************************************************************************
 * Name: z16_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void z16_ack_irq(int irq)
{
  /* System exceptions cannot be disabled or acknowledged */

  if (irq >= Z16F_IRQ_IRQ0)
    {
      /* Acknowledge the interrupt by setting the corresponding bit in the
       * IRQ status register.
       */

      if (irq < Z16F_IRQ_IRQ1)
        {
           putreg8(Z16F_IRQ0_BIT(irq), Z16F_IRQ0);
        }
      else if (irq < Z16F_IRQ_IRQ2)
        {
           putreg8(Z16F_IRQ1_BIT(irq), Z16F_IRQ2);
        }
      else if (irq < NR_IRQS)
        {
           putreg8(Z16F_IRQ2_BIT(irq), Z16F_IRQ2);
        }
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set interrupt priority
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  /* To be provided */

  return -ENOSYS;
}
#endif
