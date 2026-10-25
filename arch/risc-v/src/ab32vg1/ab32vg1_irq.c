/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_irq.c
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
#include <inttypes.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "ab32vg1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAP_EXCEPTION  0
#define TRAP_IRQ        1
#define TRAP_SYSCALL    2

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_exception
 *
 * Description:
 *   Fatal trap: illegal instruction, CSR access, ecall, ebreak or a
 *   spurious XIP cache miss.  The core does not report the cause and
 *   cannot resume after an exception (it keeps trapping on every
 *   instruction), so this never returns.
 *
 ****************************************************************************/

static void ab32vg1_exception(uintreg_t *regs)
{
  _alert("EXCEPTION: EPC: %08" PRIxREG " (address after the faulting "
         "instruction) CPU status: %08" PRIx32 "\n",
         regs[REG_EPC], getreg32(AB32VG1_CPU_STATUS));

  up_set_interrupt_context(true);
  PANIC_WITH_REGS("panic", regs);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ab32vg1_dispatch_trap
 ****************************************************************************/

uintreg_t *ab32vg1_dispatch_trap(int kind, uintreg_t *regs)
{
  uint32_t pending;
  int bit;

  switch (kind)
    {
      case TRAP_SYSCALL:
        return riscv_doirq(RISCV_IRQ_ECALLM, regs);

      case TRAP_IRQ:

        /* Dispatch one source per trap: riscv_doirq() may switch context.
         * Remaining sources trap again right after mret.
         */

        pending = getreg32(AB32VG1_PICPND) & getreg32(AB32VG1_PICEN);
        if (pending == 0)
          {
            return regs;
          }

        bit = __builtin_ctz(pending);
        if (bit == AB32VG1_IRQ_SW - AB32VG1_IRQ_FIRST)
          {
            putreg32(1 << bit, AB32VG1_PICPND);
          }

        return riscv_doirq(AB32VG1_IRQ_FIRST + bit, regs);

      default:
        ab32vg1_exception(regs);
        return regs;
    }
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all sources, all low priority, unit enabled, IE off */

  up_irq_save();
  putreg32(0, AB32VG1_PICEN);
  putreg32(0, AB32VG1_PICPR);
  putreg32(0xffffffff, AB32VG1_PICPND);
  modifyreg32(AB32VG1_PICCON, PICCON_IE, PICCON_INIT);

  /* Colorize the interrupt stack for debug purposes */

  riscv_color_intstack();

  /* The software trap is dispatched as an M-mode ecall */

  irq_attach(RISCV_IRQ_ECALLM, riscv_swint, NULL);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  irqstate_t flags;

  if (irq >= AB32VG1_IRQ_FIRST && irq < NR_IRQS)
    {
      flags = up_irq_save();
      modifyreg32(AB32VG1_PICEN, 1 << (irq - AB32VG1_IRQ_FIRST), 0);
      up_irq_restore(flags);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  irqstate_t flags;

  if (irq >= AB32VG1_IRQ_FIRST && irq < NR_IRQS)
    {
      flags = up_irq_save();
      modifyreg32(AB32VG1_PICEN, 0, 1 << (irq - AB32VG1_IRQ_FIRST));
      up_irq_restore(flags);
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags = getreg32(AB32VG1_PICCON) & PICCON_IE;

  putreg32(PICCON_IE, AB32VG1_PICCONSET);
  return flags;
}

/****************************************************************************
 * Name: riscv_get_newintctx
 *
 * Description:
 *   Return the initial REG_INT_CTX of a new thread: interrupts enabled
 *   once the thread starts (STATUS_PIE is copied to EPICCON.IE on return).
 *
 ****************************************************************************/

uintptr_t riscv_get_newintctx(void)
{
  return STATUS_PPP | STATUS_PIE;
}

/****************************************************************************
 * Name: riscv_set_idleintctx
 ****************************************************************************/

void riscv_set_idleintctx(void)
{
}
