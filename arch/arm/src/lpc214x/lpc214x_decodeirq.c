/********************************************************************************
 * arch/arm/src/lpc214x/lpc214x_decodeirq.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc214x_vic.h"

/********************************************************************************
 * Private Data
 ********************************************************************************/

/* This array maps 4 bits into the bit number of the lowest bit that it set */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
static uint8_t g_nibblemap[16] =
{
  0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0
};
#endif

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * arm_decodeirq() and/or lpc214x_decodeirq()
 *
 * Description:
 *   The vectored interrupt controller (VIC) takes 32 interrupt request inputs
 *   and pro grammatically assigns them into 3 categories:  FIQ, vectored IRQ,
 *   and non-vectored IRQ.
 *
 *   - FIQs have the highest priority.  There is a single FIQ vector, but multiple
 *     interrupt sources can be ORed to this FIQ vector.
 *
 *   - Vectored IRQs have the middle priority.  Any 16 of the 32 interrupt sources
 *     can be assigned to vectored IRQs.
 *
 *   - Non-vectored IRQs have the lowest priority.
 *
 *   The general flow of IRQ processing is to simply read the VIC vector address
 *   and jump to the address of the vector provided in the register.  The VIC will
 *   provide the address of the highest priority vectored IRQ.  If a non-vectored
 *   IRQ is requesting, the address of a default handler is provided.
 *
 ********************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
uint32_t *arm_decodeirq(uint32_t *regs)
#else
static uint32_t *lpc214x_decodeirq(uint32_t *regs)
#endif
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  CURRENT_REGS = regs;
  err("ERROR: Unexpected IRQ\n");
  PANIC();
  return NULL;
#else

  /* Decode the interrupt. We have to do this by search for the lowest numbered
   * non-zero bit in the interrupt status register.
   */

  uint32_t pending = vic_getreg(LPC214X_VIC_IRQSTATUS_OFFSET) & 0x007fffff;
  unsigned int nibble;
  unsigned int irq_base;
  unsigned int irq = NR_IRQS;

  /* Search in groups of four bits.  For 22 sources, this is at most six
   * times through the loop.
   */

  for (nibble = pending & 0x0f, irq_base = 0;
       pending && irq_base < NR_IRQS;
       pending >>= 4, nibble = pending & 0x0f, irq_base += 4)
    {
      if (nibble)
        {
          irq = irq_base + g_nibblemap[nibble];
          break;
        }
    }

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)
    {
      uint32_t *savestate;

      /* Current regs non-zero indicates that we are processing an interrupt;
       * CURRENT_REGS is also used to manage interrupt level context switches.
       */

      savestate    = (uint32_t *)CURRENT_REGS;
      CURRENT_REGS = regs;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of CURRENT_REGS.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
       */

      CURRENT_REGS = savestate;
    }

  return NULL;  /* Return not used in this architecture */
#endif
}

#ifdef CONFIG_VECTORED_INTERRUPTS
uint32_t *arm_decodeirq(uint32_t *regs)
{
  vic_vector_t vector = (vic_vector_t)vic_getreg(LPC214X_VIC_VECTADDR_OFFSET);
  vector(regs);
  return NULL;  /* Return not used in this architecture */
}
#endif
