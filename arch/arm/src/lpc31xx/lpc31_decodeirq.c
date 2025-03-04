/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_decodeirq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "arm_internal.h"
#include "sched/sched.h"
#include "lpc31_intc.h"

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
  int index;
  int irq;

  /* Read the IRQ vector status register.  Bits 3-10 provide the IRQ number
   * of the interrupt (the TABLE_ADDR was initialized to zero, so the
   * following masking should be unnecessary)
   */

  index = getreg32(LPC31_INTC_VECTOR0) & INTC_VECTOR_INDEX_MASK;
  if (index != 0)
    {
      /* Shift the index so that the range of IRQ numbers are in bits 0-7
       * (values 1-127) and back off the IRQ number by 1 so that the
       * numbering is zero-based
       */

      irq = (index >> INTC_VECTOR_INDEX_SHIFT) -1;

      /* Verify that the resulting IRQ number is valid */

      if ((unsigned)irq < NR_IRQS)
        {
          /* Acknowledge the interrupt */

          arm_ack_irq(irq);

          /* Nested interrupts are not supported. */

          DEBUGASSERT(!up_interrupt_context());

          /* Set irq flag */

          up_set_interrupt_context(true);
          tcb->xcp.regs = regs;

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

  return NULL;  /* Return not used in this architecture */
#endif
}
