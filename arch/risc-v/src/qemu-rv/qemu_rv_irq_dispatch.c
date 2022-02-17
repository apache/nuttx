/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_irq_dispatch.c
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

#include "hardware/qemu_rv_memorymap.h"
#include "hardware/qemu_rv_plic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_RV32
#  define RV_IRQ_MASK 27
#else
#  define RV_IRQ_MASK 59
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  uintptr_t irq = (vector >> RV_IRQ_MASK) | (vector & 0xf);
  uintptr_t *mepc = regs;

  if (vector < RISCV_IRQ_ECALLM)
    {
      riscv_fault(irq, regs);
    }

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      uintptr_t val = getreg32(QEMU_RV_PLIC_CLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq += val;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (RISCV_IRQ_ECALLM == irq)
    {
      *mepc += 4;
    }

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported
   */

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* MEXT means no interrupt */

  if (RISCV_IRQ_MEXT != irq)
    {
      /* Deliver the IRQ */

      irq_dispatch(irq, regs);
    }

  if (RISCV_IRQ_MEXT <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - RISCV_IRQ_MEXT, QEMU_RV_PLIC_CLAIM);
    }
#endif

  /* If a context switch occurred while processing the interrupt then
   * CURRENT_REGS may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uintptr_t *)CURRENT_REGS;
  CURRENT_REGS = NULL;

  return regs;
}
