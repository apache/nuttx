/****************************************************************************
 * arch/risc-v/src/shakti/shakti_irq_dispatch.c
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
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

// #include "riscv_arch.h"
#include "riscv_internal.h"

#include "mindgrove.h"
#include "plic.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/
void up_fault(int irq, void *regs);

volatile uintptr_t *g_current_regs[1];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

// void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
// {
//   uintptr_t  irq = (vector >> (27+32)) | (vector & 0xf);
//   uintptr_t *mepc = regs;
//   int i;

//   // This is for Normal MCAUSE, like load/store access faults.
//   if (vector < RISCV_IRQ_ECALLU)
//     {
//       up_fault((int)irq, regs);
//     }

//   // This is to check if the handler is for the interrupts, then claim the interrupt.
//   if (RISCV_IRQ_MTIMER == irq)
//   {
  
//   }

//   // This is to check if the handler is for the PLIC interrupts, then claim the interrupt.
//   if (RISCV_IRQ_MEXT == irq)
//     {
//       irq = INTERRUPT_Claim_Request()+MINDGROVE_PLIC_START;
//     }

//   /* NOTE: In case of ecall, we need to adjust mepc in the context */
//   if (RISCV_IRQ_ECALLM == irq)
//     {
//       *mepc += 4;
//     }

//   /* Acknowledge the interrupt */
//   riscv_ack_irq(irq);

// #ifdef CONFIG_SUPPRESS_INTERRUPTS
//   PANIC();
// #else
//   /* Current regs non-zero indicates that we are processing an interrupt;
//    * CURRENT_REGS is also used to manage interrupt level context switches.
//    *
//    * Nested interrupts are not supported
//    */

//   DEBUGASSERT(CURRENT_REGS == NULL);
//   CURRENT_REGS = regs;

//   /* Deliver the IRQ */

//   irq_dispatch(irq, regs);

// #endif

//   if(irq >= MINDGROVE_PLIC_START){
//     INTERRUPT_Complete(irq-MINDGROVE_PLIC_START);
//   }

//   /* If a context switch occurred while processing the interrupt then
//    * CURRENT_REGS may have change value.  If we return any value different
//    * from the input regs, then the lower level will know that a context
//    * switch occurred during interrupt processing.
//    */

//   regs = (uintptr_t *)CURRENT_REGS;
//   CURRENT_REGS = NULL;

//   return regs;
// }




void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  int irq = (vector >> (27+32)) | (vector & 0xf);

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      irq = INTERRUPT_Claim_Request()+MINDGROVE_PLIC_START;
    }

  /* Acknowledge the interrupt */

  riscv_ack_irq(irq);

  /* MEXT means no interrupt */

  if (RISCV_IRQ_MEXT != irq)
    {
      /* Deliver the IRQ */

      regs = riscv_doirq(irq, regs);
    }

  if (RISCV_IRQ_MEXT <= irq)
    {
      /* Then write PLIC_CLAIM to clear pending in PLIC */

      INTERRUPT_Complete(irq-MINDGROVE_PLIC_START);
    }

  return regs;
}