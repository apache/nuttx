/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_irq_dispatch.c
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
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dispatch_irq
 *
 * Description:
 *   In ECLIC mode mcause.EXCCODE[11:0] carries the ECLIC interrupt ID for
 *   interrupts or the exception code for exceptions.
 *
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintreg_t *regs)
{
  int irq = vector & 0xfff;

  /* If current trap is an interrupt, offset it into the async range */

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
      irq += RISCV_IRQ_ASYNC;
    }

  /* Clear the ECLIC pending bit for edge-triggered sources (the BLE radio
   * interrupts) before running the handler, so an edge that arrives while
   * the handler runs is not lost and the source does not re-fire forever.
   */

  riscv_ack_irq(irq);

  /* Deliver the IRQ */

  regs = riscv_doirq(irq, regs);

  return regs;
}
