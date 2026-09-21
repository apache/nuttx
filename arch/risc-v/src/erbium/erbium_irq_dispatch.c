/****************************************************************************
 * arch/risc-v/src/erbium/erbium_irq_dispatch.c
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
#include <nuttx/irq.h>

#include "hardware/erbium_plic.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   Dispatch hart-local traps or claim, dispatch and complete a PLIC
 *   interrupt. Complete nonzero claims even if their source is unsupported.
 *
 * Input Parameters:
 *   vector - Trap cause register value.
 *   regs - Saved registers for the interrupted task.
 *
 * Returned Value:
 *   The register context to restore, possibly belonging to another task.
 *
 ****************************************************************************/

void *riscv_dispatch_irq(uintreg_t vector, uintreg_t *regs)
{
  int irq = vector & RISCV_IRQ_MASK;
  uint32_t claim;

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
      irq += RISCV_IRQ_ASYNC;
    }

  if (irq == RISCV_IRQ_EXT)
    {
      claim = getreg32(ERBIUM_PLIC_CLAIM);
      if (claim != 0)
        {
          if (claim <= ERBIUM_PLIC_NDEV)
            {
              regs = riscv_doirq(RISCV_IRQ_EXT + claim, regs);
            }

          putreg32(claim, ERBIUM_PLIC_CLAIM);
        }
    }
  else
    {
      regs = riscv_doirq(irq, regs);
    }

  return regs;
}
