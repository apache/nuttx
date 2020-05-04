/********************************************************************************
 * arch/arm/src/dm320/dm320_decodeirq.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "group/group.h"

/********************************************************************************
 * Public Functions
 ********************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  CURRENT_REGS = regs;
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL;
#else
  /* Decode the interrupt.  First, fetch the interrupt id register. */

  uint16_t irqentry = getreg16(DM320_INTC_IRQENTRY0);

  /* The irqentry value is an offset into a table.  Zero means no interrupt. */

  if (irqentry != 0)
    {
      /* If non-zero, then we can map the table offset into an IRQ number */

      int irq = (irqentry >> 2) - 1;

      /* Verify that the resulting IRQ number is valid */

      if ((unsigned)irq < NR_IRQS)
        {
          /* Acknowledge the interrupt */

          arm_ack_irq(irq);

          /* Current regs non-zero indicates that we are processing an interrupt;
           * CURRENT_REGS is also used to manage interrupt level context switches.
           *
           * Nested interrupts are not supported.
           */

          DEBUGASSERT(CURRENT_REGS == NULL);
          CURRENT_REGS = regs;

          /* Deliver the IRQ */

          irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
          /* Check for a context switch.  If a context switch occurred, then
           * CURRENT_REGS will have a different value than it did on entry.
           * If an interrupt level context switch has occurred, then restore
           * the floating point state and the establish the correct address
           * environment before returning from the interrupt.
           */

          if (regs != CURRENT_REGS)
            {
#ifdef CONFIG_ARCH_FPU
              /* Restore floating point registers */

              arm_restorefpu((uint32_t *)CURRENT_REGS);
#endif

#ifdef CONFIG_ARCH_ADDRENV
              /* Make sure that the address environment for the previously
               * running task is closed down gracefully (data caches dump,
               * MMU flushed) and set up the address environment for the new
               * thread at the head of the ready-to-run list.
               */

              group_addrenv(NULL);
#endif
            }
#endif

          /* Set CURRENT_REGS to NULL to indicate that we are no longer in
           * an interrupt handler.
           */

          CURRENT_REGS = NULL;
        }
    }
#endif

  return NULL;  /* Return not used in this architecture */
}
