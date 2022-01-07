/****************************************************************************
 * arch/misoc/src/minerva/minerva_decodeirq.c
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

#include <arch/irq.h>

#include "chip.h"
#include "minerva.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: minerva_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in
 *   minerva_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call minerva_doirq to
 *   dispatch the interrupt.
 *
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *minerva_decodeirq(uint32_t intstat, uint32_t * regs)
{
  int irq;

  irqinfo("intstat=%08lx\n", (unsigned long)intstat);

  /* Decode and dispatch interrupts */

  for (irq = 0; irq < MINERVA_NINTERRUPTS && intstat != 0; irq++)
    {
      uint32_t bit = (1 << irq);

      /* Is this interrupt pending? */

      if ((intstat & bit) != 0)
        {
          /* Yes.. Dispatch the interrupt
           * REVISIT: Do I need to acknowledge the interrupt first?
           */

          irqinfo("irq=%d\n", irq);
          regs = minerva_doirq(irq, regs);

          /* Clear the bit in the interrupt status copy so that maybe we can
           * break out of the loop early.
           */

          intstat &= ~bit;
        }
    }

  /* Return the final task register save area.  This will typically be the
   * same as the value of regs on input.  In the event of a context switch,
   * however, it will differ.  It will refere to the register save are in
   * the TCB of the new thread.
   */

  return regs;
}
