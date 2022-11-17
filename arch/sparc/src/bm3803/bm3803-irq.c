/****************************************************************************
 * arch/sparc/src/bm3803/bm3803-irq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/irq.h>

#include "sparc_internal.h"
#include "bm3803.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static int up_prioritize_irq(int irq, int priority);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int irq;

  for (irq = 1; irq < NR_IRQS; irq++)
    {
  /* Skip window overflow, underflow, and flush as well as software
   * trap 0,9,10 which we will use as a shutdown, IRQ disable, IRQ
   * enable.Also avoid trap 0x70 - 0x7f which cannot happen and where
   * some of the space is used to pass parameters to the program.
   */

      if (((irq >= 0x00) && (irq <= 0x11)) ||
          ((irq >= 0x20) && (irq <= 0xff)))
        {
          continue;
        }

      /* Set all interrupts to the default (low) priority */

      up_prioritize_irq(irq, 0);

      /* Disable all interrupts */

      up_disable_irq(irq);
    }

  /* Attach software interrupts */

  irq_attach(BM3803_IRQ_SW_SYSCALL_TA0, sparc_swint0, NULL);
  irq_attach(BM3803_IRQ_SW_SYSCALL_TA8, sparc_swint1, NULL);

  /* And finally, enable interrupts */

  /* Interrupts are enabled by setting the te bit in the psr status
   * register
   */

  up_irq_enable();
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
  int bitno;

  /* Disable the interrupt by clearing the associated bit in the IEC
   * register
   */

  DEBUGASSERT(irq >= BM3803_IRQ_FIRST_INTERRUPT &&
              irq <= BM3803_IRQ_LAST_INTERRUPT);
  if (irq >= BM3803_IRQ_FIRST_INTERRUPT)
    {
      if (irq <= BM3803_IRQ_LAST_INTERRUPT)
        {
      /* Disable the interrupt */

          bitno = irq - BM3803_IRQ_FIRST_INTERRUPT + 1;
          BM3803_REG.int_mask &= (~(1 << bitno));
        }
      else
        {
      /* Value out of range.. just ignore */

          return;
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
  int bitno;

  /* Enable the interrupt by setting the associated bit in the Interrupt_Mask
   * Register
   */

  DEBUGASSERT(irq >= BM3803_IRQ_FIRST_INTERRUPT &&
              irq <= BM3803_IRQ_LAST_INTERRUPT);
  if (irq >= BM3803_IRQ_FIRST_INTERRUPT)
    {
      if (irq <= BM3803_IRQ_LAST_INTERRUPT)
        {
      /* Enable the interrupt */

          bitno = irq - BM3803_IRQ_FIRST_INTERRUPT + 1;
          BM3803_REG.int_mask |= (1 << bitno);
        }
      else
        {
      /* Value out of range.. just ignore */

          return;
        }
    }
}

/****************************************************************************
 * Name: sparc_pending_irq
 *
 * Description:
 *   Return true if the interrupt is pending and unmasked.
 *
 ****************************************************************************/

bool sparc_pending_irq(int irq)
{
  int bitno;
  uint16_t regval;
  uint16_t regval1;
  uint16_t regval2;

  /* Test if the interrupt is pending by reading both the IEC and IFS
   * register. Return true if the bit associated with the irq is both pending
   * the IFs and enabled in the IEC.
   */

  DEBUGASSERT(irq >= BM3803_IRQ_FIRST_INTERRUPT &&
              irq <= BM3803_IRQ_LAST_INTERRUPT);
  if (irq >= BM3803_IRQ_FIRST_INTERRUPT)
    {
      if (irq <= BM3803_IRQ_LAST_INTERRUPT)
        {
      /* Get the set of unmasked, pending interrupts.   */

          regval1 = BM3803_REG.int_mask;
          regval2 = BM3803_REG.int_pend;
          regval  = regval1 & regval2;
        }
      else
        {
      /* Value out of range.. just ignore */

          return false;
        }

      /* Return true if the interrupt is pending and unmask. */

      bitno = irq - BM3803_IRQ_FIRST_INTERRUPT + 1;
      return (regval & (1 << bitno)) != 0;
    }

  return false;
}

/****************************************************************************
 * Name: sparc_clrpend_irq
 *
 * Description:
 *   Clear any pending interrupt
 *
 ****************************************************************************/

void sparc_clrpend_irq(int irq)
{
  int bitno;
  /* Acknowledge the interrupt by clearing the associated bit in the ITP
   * register.  It is necessary to do this BEFORE lowering the interrupt
   * priority level otherwise recursive interrupts would occur.
   */

  DEBUGASSERT(irq >= BM3803_IRQ_FIRST_INTERRUPT &&
              irq <= BM3803_IRQ_LAST_INTERRUPT);
  if (irq >= BM3803_IRQ_FIRST_INTERRUPT)
    {
      if (irq <= BM3803_IRQ_LAST_INTERRUPT)
        {
      /* written with a ‘1’, in Interrupt Clear Register
       * will clear the corresponding bit(s) in the interrupt pending
       * register
       */

          bitno = irq - BM3803_IRQ_FIRST_INTERRUPT + 1;
          BM3803_REG.int_clear |= (1 << bitno);
        }
      else
        {
      /* Value out of range.. just ignore */

          return;
        }
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 * It is possible to change the priority level of an interrupt using the two
 * priority levels from the interrupt mask and priority register (ITMP).
 * Each interrupt can be assigned to one of two levels as programmed in the
 * Interrupt mask and priority register. Level 1 has higher priority than
 * level 0. Within each level the interrupts are prioritised
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static
#endif
int up_prioritize_irq(int irq, int priority)
{
  int bitno;
  int shift;

  /* Don't allow this function to be used for disabling interrupts. */

  DEBUGASSERT((unsigned)irq < NR_IRQS && (unsigned)(priority) < 2);
  if (irq >= BM3803_IRQ_FIRST_INTERRUPT)
    {
      if (irq <= BM3803_IRQ_LAST_INTERRUPT)
        {
          bitno = irq - BM3803_IRQ_FIRST_INTERRUPT + 1;
          shift  = bitno + 16;

          /* Set the new interrupt priority (momentarily disabling
           * interrupts)
           */

          BM3803_REG.int_mask &= (~(1 << shift));
          BM3803_REG.int_mask |= (priority << shift);
        }
      else
        {
          /* Value out of range.. just ignore */

          return -EINVAL;
        }
    }

  return OK;
}
