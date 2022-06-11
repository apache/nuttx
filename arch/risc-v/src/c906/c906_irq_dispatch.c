/****************************************************************************
 * arch/risc-v/src/c906/c906_irq_dispatch.c
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
#include "group/group.h"

#include "c906_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RV_IRQ_MASK 59

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  int irq = (vector >> RV_IRQ_MASK) | (vector & 0xf);

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      uint32_t val = getreg32(C906_PLIC_MCLAIM);

      /* Add the value to nuttx irq which is offset to the mext */

      irq = val + C906_IRQ_PERI_START;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

  /* MEXT means no interrupt */

  if (RISCV_IRQ_MEXT != irq)
    {
      /* Deliver the IRQ */

      regs = riscv_doirq(irq, regs);
    }

  if (C906_IRQ_PERI_START <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      putreg32(irq - C906_IRQ_PERI_START, C906_PLIC_MCLAIM);
    }

  return regs;
}
