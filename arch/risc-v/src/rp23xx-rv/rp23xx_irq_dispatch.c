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

static void *riscv_dispatch_irq_ext(uintreg_t irq, uintreg_t *regs)
{
  uint32_t meinext;
  uint32_t extirq;
  uint32_t meicontext;

  meicontext = READ_AND_SET_CSR(RVCSR_MEICONTEXT_OFFSET,
                                RVCSR_MEICONTEXT_CLEARTS_BITS);

  while (1)
    {
      meinext = READ_AND_SET_CSR(RVCSR_MEINEXT_OFFSET,
                                 RVCSR_MEINEXT_UPDATE_BITS);
      if (meinext & RVCSR_MEINEXT_NOIRQ_BITS)
        {
          break;
        }
      extirq = (meinext & RVCSR_MEINEXT_IRQ_BITS) >> 2;
      SET_CSR(CSR_MSTATUS, MSTATUS_MIE);
      regs = riscv_doirq(RP23XX_IRQ_EXTINT + extirq, regs);
      CLEAR_CSR(CSR_MSTATUS, MSTATUS_MIE);
    }

#if 0
  uint32_t ppreempt = ((meicontext >> 28) & 0xf) << 24;
  uint32_t preempt = ((meicontext >> 24) & 0xf) << 16;
  uint32_t parentirq = meicontext & 0xc;
  WRITE_CSR(RVCSR_MEICONTEXT_OFFSET, ppreempt | preempt | parentirq);
#endif

  WRITE_CSR(RVCSR_MEICONTEXT_OFFSET, meicontext);
  return regs;
}

static void *riscv_dispatch_async_irq(uintreg_t irq, uintreg_t *regs)
{
  irq += RISCV_IRQ_ASYNC;

  if (irq == RISCV_IRQ_EXT)
    {
      regs = riscv_dispatch_irq_ext(irq, regs);
    }
  else
    {
      regs = riscv_doirq(irq, regs);
    }
  return regs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintreg_t vector, uintreg_t *regs)
{
  int irq = vector & (~RISCV_IRQ_BIT);

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
      regs = riscv_dispatch_async_irq(irq, regs);
    }
  else
    {
      regs = riscv_doirq(irq, regs);
    }

  return regs;
}
