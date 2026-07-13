/****************************************************************************
 * arch/mips/src/jz4780/jz4780_decodeirq.c
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

#include <debug.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mips_internal.h"

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
 * Name: jz4780_decodeirq
 *
 * Description:
 *   Called from assembly language logic when an interrupt exception occurs.
 *   This function decodes and dispatches the interrupt.
 *
 ****************************************************************************/

uint32_t *jz4780_decodeirq(uint32_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];

  if (*running_task != NULL)
    {
      mips_copystate((*running_task)->xcp.regs, regs);
    }

  unsigned int source = getreg32(ICPR0);
  unsigned int source1 = getreg32(ICPR1);

  int irq = -1;
  if (source != 0)
    {
      irq = 31 - __builtin_clz(source);
    }
  else if (source1 != 0)
    {
      irq = 32 + 31 - __builtin_clz(source1);
    }

  /* If the board supports LEDs, turn on an LED now to indicate that we are
   * processing an interrupt.
   */

  board_autoled_on(LED_INIRQ);

  /* Save the current value of g_current_regs (to support nested interrupt
   * handling).  Then set g_current_regs to regs, indicating that this is
   * the interrupted context that is being processed now.
   */

  DEBUGASSERT(up_current_regs() == NULL);
  up_set_current_regs(regs);

  if (irq == JZ4780_IRQ_TCU1)
    {
      putreg32(TFCR_FFCL5, TFCR);
    }
  else if (irq == JZ4780_IRQ_TCU2)
    {
      uint32_t tfcr = getreg32(TFR) & 0xdf;
      if (!tfcr)
        {
          irq = -1;
        }
      else
        {
          int n = 31 - __builtin_clz(tfcr);
          putreg32(TFCR_FFCL(n), TFCR);
          irq = IRQ_TMR0 + n;
        }
    }

  if (irq >= 0)
    {
      /* Deliver the IRQ */

      irq_dispatch(irq, regs);
    }

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = up_current_regs();

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != up_current_regs())
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      up_restorefpu(up_current_regs());
#endif

#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      (void)group_addrenv(NULL);
#endif
    }
#endif

  up_set_current_regs(NULL);
  board_autoled_off(LED_INIRQ);

  return regs;
}

/****************************************************************************
 * Name: jz4780_swint0
 *
 * Description:
 *   Called from assembly language logic when an interrupt exception occurs.
 *   Software interrupts are unused on this platform
 *
 ****************************************************************************/

uint32_t *jz4780_swint0(uint32_t *regs)
{
  /* Leave this here so we can test exception handler placement */

  _alert("Unexpected software interrupt\n");
  return regs;
}

