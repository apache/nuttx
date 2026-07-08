/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_irq.c
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

#include <nuttx/debug.h>
#include <nuttx/arch.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"
#include "lpc214x_vic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int reg;

  /* Disable all interrupts.  We do this by writing zero to the IntEnable
   * register.  This is equivalent to writing all ones to the IntClearEnable
   * register.
   */

  vic_putreg(0, LPC214X_VIC_INTENABLE_OFFSET);

  /* Select all IRQs, no FIQs */

  vic_putreg(0, LPC214X_VIC_INTSELECT_OFFSET);

  /* Set the default vector */

  vic_putreg((uint32_t)arm_decodeirq, LPC214X_VIC_DEFVECTADDR_OFFSET);

  /* Disable all vectored interrupts */

  for (reg = LPC214X_VIC_VECTCNTL0_OFFSET;
       reg <= LPC214X_VIC_VECTCNTL15_OFFSET;
       reg += 4)
    {
      vic_putreg(0, reg);
    }

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm_color_intstack();
  up_irq_restore(PSR_MODE_SYS | PSR_F_BIT);
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
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable the irq by setting the corresponding bit in the VIC
       * Interrupt Enable Clear register.
       */

      vic_putreg((1 << irq), LPC214X_VIC_INTENCLEAR_OFFSET);
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
  /* Verify that the IRQ number is within range */

  if (irq < NR_IRQS)
    {
      /* Disable all interrupts */

      irqstate_t flags = enter_critical_section();

      /* Enable the irq by setting the corresponding bit in the VIC
       * Interrupt Enable register.
       */

      uint32_t val = vic_getreg(LPC214X_VIC_INTENABLE_OFFSET);
      vic_putreg(val | (1 << irq), LPC214X_VIC_INTENABLE_OFFSET);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  uint32_t reg32;

  if ((unsigned)irq < NR_IRQS)
    {
      /* Mask the IRQ by clearing the associated bit in Software Priority
       * Mask register
       */

      reg32 = vic_getreg(LPC214X_VIC_PRIORITY_MASK_OFFSET);
      reg32 &= ~(1 << irq);
      vic_putreg(reg32, LPC214X_VIC_PRIORITY_MASK_OFFSET);
    }

  /* Clear interrupt */

  vic_putreg((1 << irq), LPC214X_VIC_SOFTINTCLEAR_OFFSET);
  vic_putreg(0, LPC214X_VIC_ADDRESS_OFFSET);    /* dummy write to clear VICADDRESS */
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   set interrupt priority
 * MOD
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  /* The default priority on reset is 16 */

  if (irq < NR_IRQS && priority > 0 && priority < 16)
    {
      int offset = irq << 2;
      vic_putreg(priority, LPC214X_VIC_VECTPRIORITY0_OFFSET + offset);
      return OK;
    }

  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: up_attach_vector
 *
 * Description:
 *   Attach a user-supplied handler to a vectored interrupt
 *
 ****************************************************************************/

void up_attach_vector(int irq, int vector, vic_vector_t handler)
{
  /* Verify that the IRQ number and vector number are within range */

  if (irq < NR_IRQS && vector < 16 && handler)
    {
      int offset = vector << 2;

      /* Disable all interrupts */

      irqstate_t flags = enter_critical_section();

      /* Save the vector address */

      vic_putreg((uint32_t)handler, LPC214X_VIC_VECTADDR0_OFFSET + offset);

      /* Enable the vectored interrupt */

      vic_putreg(((irq << LPC214X_VECTCNTL_IRQSHIFT) |
                  LPC214X_VECTCNTL_ENABLE),
                  LPC214X_VIC_VECTCNTL0_OFFSET + offset);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: up_detach_vector
 *
 * Description:
 *   Detach a user-supplied handler from a vectored interrupt
 *
 ****************************************************************************/

void up_detach_vector(int vector)
{
  /* Verify that the vector number is within range */

  if (vector < 16)
    {
      /* Disable the vectored interrupt */

      int offset = vector << 2;
      vic_putreg(0, LPC214X_VIC_VECTCNTL0_OFFSET + offset);
    }
}
