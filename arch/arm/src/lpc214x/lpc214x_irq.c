/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_irq.c
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
#include <debug.h>
#include <nuttx/arch.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"
#include "lpc214x_vic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
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
 * Name: up_attach_vector
 *
 * Description:
 *   Attach a user-supplied handler to a vectored interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
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
#endif

/****************************************************************************
 * Name: up_detach_vector
 *
 * Description:
 *   Detach a user-supplied handler from a vectored interrupt
 *
 ****************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
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
#endif
