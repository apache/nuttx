/****************************************************************************
 * arch/arm/src/armv8-m/arm_doirq.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* Nested interrupts are not supported in this implementation.  If you
   * want to implement nested interrupts, you would have to (1) change the
   * way that CURRENT_REGS is handled and (2) the design associated with
   * CONFIG_ARCH_INTERRUPTSTACK.
   */

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  if (CURRENT_REGS == NULL)
    {
      CURRENT_REGS = regs;
      regs         = NULL;
    }

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  /* Deliver the IRQ */

  irq_dispatch(irq, (uint32_t *)CURRENT_REGS);

  /* If a context switch occurred while processing the interrupt then
   * CURRENT_REGS may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  if (regs == NULL)
    {
      /* Restore the cpu lock */

      if (regs != CURRENT_REGS)
        {
          restore_critical_section();
          regs = (uint32_t *)CURRENT_REGS;
        }

      /* Update the CURRENT_REGS to NULL. */

      CURRENT_REGS = NULL;
    }
#endif

  board_autoled_off(LED_INIRQ);
  return regs;
}
