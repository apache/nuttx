/****************************************************************************
 * arch/arm/src/dm320/dm320_decodeirq.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>

#include "chip.h"
#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL;
#else
  /* Decode the interrupt.  First, fetch the interrupt id register. */

  uint16_t irqentry = getreg16(DM320_INTC_IRQENTRY0);

  /* The irqentry value is an offset into a table.
   * Zero means no interrupt.
   */

  if (irqentry != 0)
    {
      /* If non-zero, then we can map the table offset into an IRQ number */

      int irq = (irqentry >> 2) - 1;

      /* Verify that the resulting IRQ number is valid */

      if ((unsigned)irq < NR_IRQS)
        {
          /* Acknowledge the interrupt */

          arm_ack_irq(irq);

          /* Nested interrupts are not supported. */

          DEBUGASSERT(!up_interrupt_context());
          tcb->xcp.regs = regs;

          /* Set irq flag */

          up_set_interrupt_context(true);

          /* Deliver the IRQ */

          irq_dispatch(irq, regs);
          tcb = this_task();

#ifdef CONFIG_ARCH_ADDRENV
          /* Check for a context switch. */

          if (regs != tcb->xcp.regs)
            {
              /* Make sure that the address environment for the previously
               * running task is closed down gracefully (data caches dump,
               * MMU flushed) and set up the address environment for the new
               * thread at the head of the ready-to-run list.
               */

              addrenv_switch(tcb);
            }
#endif

          /* Set irq flag */

          up_set_interrupt_context(false);
        }
    }
#endif

  return NULL;  /* Return not used in this architecture */
}
