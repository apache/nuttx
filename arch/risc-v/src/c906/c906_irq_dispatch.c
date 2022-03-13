/****************************************************************************
 * arch/risc-v/src/c906/c906_irq_dispatch.c
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

#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uint64_t vector, uint64_t *regs)
{
  uint32_t  irq = (vector >> (27 + 32)) | (vector & 0xf);
  uint64_t *mepc = regs;

  /* Check if fault happened */

  if (vector < RISCV_IRQ_ECALLU)
    {
      riscv_fault((int)irq, regs);
    }

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      uint32_t val = getreg32(C906_PLIC_MCLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq = val + C906_IRQ_PERI_START;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (RISCV_IRQ_ECALLM == irq || RISCV_IRQ_ECALLU == irq)
    {
      *mepc += 4;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported
   */

  ASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* MEXT means no interrupt */

  if (RISCV_IRQ_MEXT != irq)
    {
      /* Deliver the IRQ */

      irq_dispatch(irq, regs);
    }

  if (C906_IRQ_PERI_START <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - C906_IRQ_PERI_START, C906_PLIC_MCLAIM);
    }

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * CURRENT_REGS will have a different value than it did on entry.  If an
   * interrupt level context switch has occurred, then restore the floating
   * point state and the establish the correct address environment before
   * returning from the interrupt.
   */

  if (regs != CURRENT_REGS)
    {
#ifdef CONFIG_ARCH_FPU
      /* Restore floating point registers */

      riscv_restorefpu((uint64_t *)CURRENT_REGS);
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

#endif

  /* If a context switch occurred while processing the interrupt then
   * CURRENT_REGS may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint64_t *)CURRENT_REGS;
  CURRENT_REGS = NULL;

  return regs;
}
