/****************************************************************************
 * arch/risc-v/src/k210/k210_irq_dispatch.c
 *
 *   Copyright (C) 2019 Masayuki Ishikawa. All rights reserved.
 *   Author: Masayuki Ishikawa <masayuki.ishikawa@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Public Data
 ****************************************************************************/

extern void up_fault(int irq, uint64_t *regs);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * k210_dispatch_irq
 ****************************************************************************/

void *k210_dispatch_irq(uint64_t vector, uint64_t *regs)
{
  uint32_t  irq = (vector >> (27 + 32)) | (vector & 0xf);
  uint64_t *mepc = regs;

  /* Check if fault happened */

  if (vector < K210_IRQ_ECALLU)
    {
      up_fault((int)irq, regs);
    }

  /* Firstly, check if the irq is machine external interrupt */

  if (K210_IRQ_MEXT == irq)
    {
      uint32_t val = getreg32(K210_PLIC_CLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq += val;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (K210_IRQ_ECALLM == irq || K210_IRQ_ECALLU == irq)
    {
      *mepc += 4;
    }

  /* Acknowledge the interrupt */

  up_ack_irq(irq);

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

  if (K210_IRQ_MEXT != irq)
    {
      /* Deliver the IRQ */

      irq_dispatch(irq, regs);
    }

  if (K210_IRQ_MEXT <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - K210_IRQ_MEXT, K210_PLIC_CLAIM);
    }
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
