/****************************************************************************
 * arch/risc-v/src/rp23xx-rv/rp23xx_irq_dispatch.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "rp23xx_gpio.h"
#include "hardware/rp23xx_hazard3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintreg_t mcause, uintreg_t *regs)
{
  int  irq;
  bool is_irq = (RISCV_IRQ_BIT & mcause) != 0;

  if (is_irq)
    {
      irq = mcause & RISCV_IRQ_MASK;
      irq += RISCV_IRQ_ASYNC;
      if (RISCV_IRQ_MEXT == irq)
        {
          uint32_t val = (READ_CSR(RVCSR_MEINEXT_OFFSET) >> 2) & 0x1ff;
          irq = RP23XX_IRQ_EXTINT + val;
        }
    }
    else
    {
      irq = mcause;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

  /* Deliver the IRQ */

  regs = riscv_doirq(irq, regs);

  return regs;
}
