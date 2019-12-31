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

#include "up_arch.h"
#include "up_internal.h"

#include "group/group.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint64_t * g_current_regs;

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

  if (K210_IRQ_MEXT == irq)
    {
      /* Read & write K210_PLIC_CLAIM to clear pending */

      uint32_t val = getreg32(K210_PLIC_CLAIM);
      putreg32(val, K210_PLIC_CLAIM);

      irq += val;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (K210_IRQ_ECALLM == irq)
    {
      *mepc += 4;
    }

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else
  /* Nested interrupts are not supported */

  DEBUGASSERT(g_current_regs == NULL);

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  g_current_regs = regs;

  /* Deliver the IRQ */

  irq_dispatch(irq, regs);
#endif

  /* If a context switch occurred while processing the interrupt then
   * g_current_regs may have change value.  If we return any value different
   * from the input regs, then the lower level will know that a context
   * switch occurred during interrupt processing.
   */

  regs = (uint64_t *)g_current_regs;
  g_current_regs = NULL;

  /* Set machine previous privilege mode to machine mode */

  *(regs + REG_INT_CTX_NDX) |= 0x3 << 11;

  return regs;
}

