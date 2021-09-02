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

#include "riscv_internal.h"
#include "rv32m1.h"
#include "hardware/rv32m1_eu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RV_IRQ_MASK 27

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * rv32m1_dispatch_irq
 ****************************************************************************/

LOCATE_ITCM
void *rv32m1_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  uint32_t vec = vector & 0x1f;
  int irq = (vector >> RV_IRQ_MASK) + vec;
  int irqofs = 0;

  if (RV32M1_IRQ_INTMUX0 <= irq)
    {
      uintptr_t chn = irq - RV32M1_IRQ_INTMUX0;
      uintptr_t regaddr = RV32M1_INTMUX_CH_BASE(chn) + INTMUX_CH_VEC_OFFSET;
      uintptr_t regval = getreg32(regaddr);

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

  /* Deliver the IRQ */

  regs = riscv_doirq(irq, regs);

  if (RV32M1_IRQ_MEXT <= irq)
    {
      irq -= irqofs;

      /* Clear the pending flag */

      putreg32(1 << vec, RV32M1_EU_INTPTPENDCLR);
    }

  return regs;
}
