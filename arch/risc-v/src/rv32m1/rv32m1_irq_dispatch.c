/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_irq_dispatch.c
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

#include "rv32m1.h"
#include "hardware/rv32m1_eu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t * g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * rv32m1_dispatch_irq
 ****************************************************************************/

LOCATE_ITCM
void *rv32m1_dispatch_irq(uint32_t vector, uint32_t *regs)
{
  int vec = vector & 0x1f;
  int irq = (vector >> 27) + vec;
  uint32_t *mepc = regs;

  int irqofs = 0;

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (RV32M1_IRQ_ECALL_M == irq)
    {
      *mepc += 4;
    }

  if (RV32M1_IRQ_INTMUX0 <= irq)
    {
      uint32_t const chn = irq - RV32M1_IRQ_INTMUX0;
      uint32_t regaddr = RV32M1_INTMUX_CH_BASE(chn) + INTMUX_CH_VEC_OFFSET;
      uint32_t regval = getreg32(regaddr);

      /* CH_VEC coudle be 0 while INTMUX doesn't latch pending source
       * interrupts. In that case a spurious interrupt is being serviced,
       * and irq Number shouldn't be compensated.
       *
       * CH_VEC must be checked to account for spurious interrupts.
       */

      if (regval > 0)
        {
          /* Register VECN[13:2] = 48 x (CPU Vectors + NVIC Vectors) +
           *                     H(The Highest Interrupt of INTMUX),
           *
           * 1 CPU Vectors for RV32M1 RISCV Cores.
           * No NVIC Vectors for RV32M1 RISCV Cores,
           *
           * H can be obtained easily:
           * H = VECN[13:2] - 48
           *
           * H has to be offset by 8 to skip INTMUX0~7.
           *
           */

          irqofs = (regval >> 2) - 48 + 8;
          irq += irqofs;
        }
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

  if (RV32M1_IRQ_MEXT <= irq)
    {
      irq -= irqofs;

      /* Clear the pending flag */

      putreg32(1 << vec, RV32M1_EU_INTPTPENDCLR);
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
