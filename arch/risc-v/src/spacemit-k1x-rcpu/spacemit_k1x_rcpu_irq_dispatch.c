/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_irq_dispatch.c
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
#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   Dispatch a trap that the assembly entry has decoded.
 *
 *   The K1 RCPU runs its ECLIC in CLIC mode, which changes two things
 *   compared with the PLIC/CLINT cores elsewhere in NuttX:
 *
 *   1. There is no claim/complete register.  The identity of the source
 *      arrives in mcause.EXCCODE[11:0], so the low 12 bits of the vector
 *      handed to this function are the ECLIC source id, not an index that
 *      has to be looked up.
 *
 *   2. Per-source gating lives in the ECLIC INTIE registers.  The mie CSR
 *      only carries the architecturally defined software and timer bits.
 *
 * Input Parameters:
 *   vector - The mcause value captured by the trap entry
 *   regs   - The register save area built by the trap entry
 *
 * Returned Value:
 *   The (possibly switched) register save area
 *
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintreg_t *regs)
{
  int irq = vector & 0xfff;

  /* If the trap is an interrupt, offset it into the async range.  Exceptions
   * keep the architecturally defined small codes and are handled by the
   * common exception path.
   */

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
      irq += RISCV_IRQ_ASYNC;
    }

  /* Acknowledge before running the handler.  A level triggered source stays
   * asserted until its driver services the peripheral, so clearing the
   * pending bit is a no-op for it; an edge triggered source would otherwise
   * re-fire for an edge that arrives while the handler runs.
   */

  riscv_ack_irq(irq);

  /* Deliver the IRQ */

  regs = riscv_doirq(irq, regs);

  return regs;
}
