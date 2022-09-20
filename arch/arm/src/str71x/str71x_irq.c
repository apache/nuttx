/****************************************************************************
 * arch/arm/src/str71x/str71x_irq.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"
#include "str71x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* The bulk of IRQ initialization if performed in str71x_head.S, so we
   * have very little to do here -- basically just enabling interrupts;
   *
   * Enable IRQs (but not FIQs -- they aren't used)
   */

  putreg32(STR71X_EICICR_IRQEN, STR71X_EIC_ICR);

  /* This shouldn't be necessary, but it appears that something is needed
   * here to prevent spurious interrupts when the ARM interrupts are enabled
   * (Needs more investigation).
   */

  up_mdelay(50);                        /* Wait a bit */
#if 0
  putreg32(0, STR71X_EIC_IER);          /* Make sure that all interrupts are disabled */
  putreg32(0xffffffff, STR71X_EIC_IPR); /* And that no interrupts are pending */
#endif

  /* Initialize external interrupts */

  str71x_xtiinitialize();

  /* Enable global ARM interrupts */

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
  uint32_t reg32;

  if ((unsigned)irq < STR71X_NBASEIRQS)
    {
      /* Mask the IRQ by clearing the associated bit in the IER register */

      reg32  = getreg32(STR71X_EIC_IER);
      reg32 &= ~(1 << irq);
      putreg32(reg32, STR71X_EIC_IER);
    }
#ifdef CONFIG_STR71X_XTI
  else if ((unsigned)irq < NR_IRQS)
    {
      str71x_disable_xtiirq(irq);
    }
#endif /* CONFIG_STR71X_XTI */
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
  uint32_t reg32;

  if ((unsigned)irq < STR71X_NBASEIRQS)
    {
      /* Enable the IRQ by setting the associated bit in the IER register */

      reg32  = getreg32(STR71X_EIC_IER);
      reg32 |= (1 << irq);
      putreg32(reg32, STR71X_EIC_IER);
    }
#ifdef CONFIG_STR71X_XTI
  else if ((unsigned)irq < NR_IRQS)
    {
      str71x_enable_xtiirq(irq);
    }
#endif /* CONFIG_STR71X_XTI */
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt.  No XTI support.. only used in interrupt
 *   handling logic.
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  uint32_t reg32;

  if ((unsigned)irq < STR71X_NBASEIRQS)
    {
      /* Clear the interrupt by writing a one to the corresponding bit in the
       * IPR register.
       */

      reg32  = getreg32(STR71X_EIC_IPR);
      reg32 |= (1 << irq);
      putreg32(reg32, STR71X_EIC_IPR);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set interrupt priority.  Note, there is no way to prioritize
 *   individual XTI interrupts.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t addr;
  uint32_t reg32;

  /* The current interrupt priority (CIP) is always zero, so a minimum
   * prioriy of one is enforced to prevent disabling the interrupt.
   */

  if ((unsigned)irq < STR71X_NBASEIRQS && priority > 0 && priority < 16)
    {
      addr   = STR71X_EIC_SIR(irq);
      reg32  = getreg32(addr);
      reg32 &= ~STR71X_EICSIR_SIPLMASK;
      reg32 |= priority;
      putreg32(reg32, addr);
      return OK;
    }

  return -EINVAL;
}
#endif
