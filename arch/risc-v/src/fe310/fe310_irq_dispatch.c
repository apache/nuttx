/****************************************************************************
 * arch/risc-v/src/fe310/fe310_irq_dispatch.c
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

#include "fe310_gpio.h"
#include "fe310.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t * g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * fe310_dispatch_irq
 ****************************************************************************/

void *fe310_dispatch_irq(uint32_t vector, uint32_t *regs)
{
  uint32_t  irq = (vector >> 27) | (vector & 0xf);
  uint32_t *mepc = regs;

  /* Firstly, check if the irq is machine external interrupt */

  if (FE310_IRQ_MEXT == irq)
    {
      uint32_t val = getreg32(FE310_PLIC_CLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq += val;
    }

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (FE310_IRQ_ECALLM == irq)
    {
      *mepc += 4;
    }

  /* Acknowledge the interrupt */

  up_ack_irq(irq);

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

  if (FE310_IRQ_MEXT <= irq)
    {
      /* If the irq is from GPIO, clear pending bit in the GPIO */

      if (FE310_IRQ_GPIO0 <= irq && irq <= FE310_IRQ_GPIO31)
        {
          fe310_gpio_clearpending(irq - FE310_IRQ_GPIO0);
        }

      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - FE310_IRQ_MEXT, FE310_PLIC_CLAIM);
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
