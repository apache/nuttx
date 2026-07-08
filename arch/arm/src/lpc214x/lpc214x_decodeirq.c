/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_decodeirq.c
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
#include <assert.h>
#include <nuttx/debug.h>

#include "chip.h"
#include "arm_internal.h"
#include "lpc214x_vic.h"
#include "sched/sched.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * arm_decodeirq() and/or lpc214x_decodeirq()
 *
 * Description:
 *   The vectored interrupt controller (VIC) takes 32 interrupt request
 *   inputs and pro grammatically assigns them into 3 categories:  FIQ,
 *   vectored IRQ, and non-vectored IRQ.
 *
 *   - FIQs have the highest priority.  There is a single FIQ vector, but
 *     multiple interrupt sources can be ORed to this FIQ vector.
 *
 *   - Vectored IRQs have the middle priority.  Any 16 of the 32 interrupt
 *     sources can be assigned to vectored IRQs.
 *
 *   - Non-vectored IRQs have the lowest priority.
 *
 *   The general flow of IRQ processing is to simply read the VIC vector
 *   address and jump to the address of the vector provided in the register.
 *   The VIC will provide the address of the highest priority vectored IRQ.
 *   If a non-vectored IRQ is requesting, the address of a default handler
 *   is provided.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  err("ERROR: Unexpected IRQ\n");
  PANIC();
#else

  /* Check which IRQ fires */

  uint32_t irqbits = vic_getreg(LPC214X_VIC_IRQSTATUS_OFFSET) & 0xffffffff;
  unsigned int irq;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      if (irqbits & (uint32_t) (1 << irq))
        {
          break;
        }
    }

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)
    {
      /* Deliver the IRQ */

      regs = arm_doirq(irq, regs);
    }
#endif

  return regs;
}
