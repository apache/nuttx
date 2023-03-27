/****************************************************************************
 * arch/risc-v/src/litex/litex_irq_dispatch.c
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
#include "litex.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RV_IRQ_MASK 27

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
#ifdef CONFIG_LITEX_CORE_VEXRISCV_SMP
  int irq = (vector & 0x3f);

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
       irq += RISCV_IRQ_ASYNC;
    }

  /* Firstly, check if the irq is machine external interrupt */

  if (irq == RISCV_IRQ_EXT)
    {
      uint32_t ext = getreg32(LITEX_PLIC_CLAIM);

      /* Add the value to nuttx irq which is offset to the ext */

      irq = RISCV_IRQ_EXT + ext;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

  /* EXT means no interrupt */

  if (irq != RISCV_IRQ_EXT)
    {
      /* Deliver the IRQ */

      regs = riscv_doirq(irq, regs);
    }

  if (irq > RISCV_IRQ_EXT)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - RISCV_IRQ_EXT, LITEX_PLIC_CLAIM);
    }

#else 
  int i;
  int irq = (vector >> RV_IRQ_MASK) | (vector & 0xf);

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      /* litex vexriscv dont follow riscv plic standard */

      unsigned int pending;
      unsigned int mask;
      asm volatile ("csrr %0, %1" : "=r"(pending) : "i"(LITEX_MPENDING_CSR));
      asm volatile ("csrr %0, %1" : "=r"(mask) : "i"(LITEX_MMASK_CSR));

      uint32_t val = (pending & mask);
      for (i = 0; i < 32; i++)
        {
          if (val & (1 << i))
            {
              val = i;
              val++;
              break;
            }
        }

      /* Add the value to nuttx irq which is offset to the mext */

      irq += val;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

  /* Deliver the IRQ */

  regs = riscv_doirq(irq, regs);

#endif /* CONFIG_LITEX_CORE_VEXRISCV_SMP */
  return regs;
}
