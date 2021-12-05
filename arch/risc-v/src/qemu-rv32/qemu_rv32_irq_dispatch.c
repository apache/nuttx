/****************************************************************************
 * arch/risc-v/src/qemu-rv32/qemu_rv32_irq_dispatch.c
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

#include "hardware/qemu_rv32_memorymap.h"
#include "hardware/qemu_rv32_plic.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t * g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * qemu_rv32_dispatch_irq
 ****************************************************************************/

void *qemu_rv32_dispatch_irq(uint32_t vector, uint32_t *regs)
{
  uint32_t  irq = (vector >> 27) | (vector & 0xf);
  uint32_t *mepc = regs;

  /* Firstly, check if the irq is machine external interrupt */

  if (QEMU_RV32_IRQ_MEXT == irq)
    {
      uint32_t val = getreg32(QEMU_RV32_PLIC_CLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq += val;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (QEMU_RV32_IRQ_ECALLM == irq)
    {
      *mepc += 4;
    }

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

  if (QEMU_RV32_IRQ_MEXT <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - QEMU_RV32_IRQ_MEXT, QEMU_RV32_PLIC_CLAIM);
    }
#endif

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint32_t *)g_current_regs;
  g_current_regs = NULL;

  return regs;
}
