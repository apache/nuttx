/********************************************************************************
 * arch/arm/src/lpc31xx/lpc31_decodeirq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "arm_arch.h"

#include "arm_internal.h"
#include "group/group.h"

#include "lpc31_intc.h"

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
  int index;
  int irq;

  /* Read the IRQ vector status register.  Bits 3-10 provide the IRQ number
   * of the interrupt (the TABLE_ADDR was initialized to zero, so the
   * following masking should be unnecessary)
   */

  index = getreg32(LPC31_INTC_VECTOR0) & INTC_VECTOR_INDEX_MASK;
  if (index != 0)
    {
      /* Shift the index so that the range of IRQ numbers are in bits 0-7 (values
       * 1-127) and back off the IRQ number by 1 so that the numbering is zero-based
       */

      irq = (index >> INTC_VECTOR_INDEX_SHIFT) -1;

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
          /* Set CURRENT_REGS to NULL to indicate that we are no longer in an
           * interrupt handler.
           */

          CURRENT_REGS = NULL;
        }
    }

  return NULL;  /* Return not used in this architecture */
#endif
}
