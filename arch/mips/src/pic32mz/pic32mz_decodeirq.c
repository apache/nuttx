/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_decodeirq.c
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

#include  <debug.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mips_internal.h"
#include "hardware/pic32mz_int.h"

#include "group/group.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: pic32mz_decodeirq
 *
 * Description:
 *   Called from assembly language logic when an interrupt exception occurs.
 *   This function decodes and dispatches the interrupt.
 *
 ****************************************************************************/

uint32_t *pic32mz_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
  uint32_t *savestate;
#endif
  uint32_t regval;
  int irq;

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

  /* Save the current value of g_current_regs (to support nested interrupt
   * handling).  Then set g_current_regs to regs, indicating that this is
   * the interrupted context that is being processed now.
   */

#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
  savestate = (uint32_t *)CURRENT_REGS;
#else
  DEBUGASSERT(CURRENT_REGS == NULL);
#endif
  CURRENT_REGS = regs;

  /* Loop while there are pending interrupts with priority greater than
   * zero
   */

  for (; ; )
    {
      /* Read the INTSTAT register.  This register contains both the priority
       * and the interrupt vector number.
       */

      regval = getreg32(PIC32MZ_INT_INTSTAT);
      if ((regval & INT_INTSTAT_SRIPL_MASK) == 0)
        {
          /* Break out of the loop when the priority is zero meaning that
           * there are no further pending interrupts.
           */

          break;
        }

      /* Get the vector number.  The IRQ numbers have been arranged so that
       * vector numbers and NuttX IRQ numbers are the same value.
       */

      irq = ((regval) & INT_INTSTAT_SIRQ_MASK) >> INT_INTSTAT_SIRQ_SHIFT;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);
    }

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint32_t *)CURRENT_REGS;

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      up_restorefpu((uint32_t *)CURRENT_REGS);
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

#ifdef CONFIG_PIC32MZ_NESTED_INTERRUPTS
  /* Restore the previous value of g_current_regs.  NULL would indicate that
   * we are no longer in an interrupt handler.  It will be non-NULL if we
   * are returning from a nested interrupt.
   *
   * REVISIT: There are task switching issues!  You should not enable
   * nested interrupts unless you are ready to deal with the complexities
   * of fixing nested context switching.  The logic here is insufficient.
   */

  CURRENT_REGS = savestate;
  if (CURRENT_REGS == NULL)
    {
      board_autoled_off(LED_INIRQ);
    }
#else
  CURRENT_REGS = NULL;
  board_autoled_off(LED_INIRQ);
#endif

  return regs;
}
