/****************************************************************************
 * arch/arm/src/str71x/str71x_decodeirq.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * arm_decodeirq()
 *
 * Description:
 *   Read the IRQ number from the IVR register.  During initialization, the
 *   IVR register was set to zero.  Each SIR[n] register was programmed to
 *   contain the IRQ number.  At IRQ processing time (when this function
 *   run), the IVR should contain the desired IRQ number.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  board_autoled_on(LED_INIRQ);

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL;
#else
  unsigned int irq;

  /* Read the IRQ number from the IVR register (Could probably get the same
   * info from CIC register without the setup).
   */

  board_autoled_on(LED_INIRQ);
  irq = getreg32(STR71X_EIC_IVR);

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)
    {
      uint32_t *saveregs;
      bool savestate;

      savestate = up_interrupt_context();
      saveregs = tcb->xcp.regs;
      up_set_interrupt_context(true);
      tcb->xcp.regs = regs;

      /* Acknowledge the interrupt */

      arm_ack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of saveregs. */

      up_set_interrupt_context(savestate);
      tcb->xcp.regs = saveregs;
    }
#ifdef CONFIG_DEBUG_FEATURES
  else
    {
      DEBUGPANIC(); /* Normally never happens */
    }
#endif

  board_autoled_off(LED_INIRQ);
  return NULL;  /* Return not used in this architecture */
#endif
}
