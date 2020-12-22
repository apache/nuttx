/****************************************************************************
 * arch/risc-v/src/bl602/bl602_irq_dispatch.c
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

#include "riscv_arch.h"
#include "riscv_internal.h"

#include "chip.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * bl602_dispatch_irq
 ****************************************************************************/

void *bl602_dispatch_irq(uint32_t vector, uint32_t *regs)
{
  uint32_t  irq  = vector & 0x3ff; /* E24 [9:0] */
  uint32_t *mepc = regs;

  /* If current is interrupt */

  if (vector & 0x80000000u)
    {
      irq += BL602_IRQ_ASYNC;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (BL602_IRQ_ECALLM == irq)
    {
      *mepc += 4;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Current regs non-zero indicates that we are processing an interrupt;
   * g_current_regs is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported
   */

  DEBUGASSERT(g_current_regs == NULL);
  g_current_regs = regs;

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * g_current_regs will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != g_current_regs)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      riscv_restorefpu((uint32_t *)g_current_regs);
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
#endif /* CONFIG_ARCH_FPU || CONFIG_ARCH_ADDRENV */

#endif /* CONFIG_SUPPRESS_INTERRUPTS */

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs           = (uint32_t *)g_current_regs;
  g_current_regs = NULL;

  return regs;
}
