/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_decodeirq.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2010 Rommel Marcelo. All rights reserved.
 * SPDX-FileCopyrightText: 2010,2011 Gregory Nutt. All rights reserved.
 * SPDX-FileContributor: Rommel Marcelo
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc2378.h"
#include "lpc23xx_vic.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * arm_decodeirq() and/or lpc23xx_decodeirq()
 *
 * Description:
 *   The vectored interrupt controller (VIC) takes 32 interrupt request
 *   inputs and pro grammatically assigns them into 2 categories:  FIQ,
 *   vectored IRQ.
 *
 *   - FIQs have the highest priority.  There is a single FIQ vector, but
 *     multiple interrupt sources can be ORed to this FIQ vector.
 *
 *   - Vectored IRQs have the middle priority.  Any of the 32 interrupt
 *     sources can be assigned to vectored IRQs.
 *
 *   - Non-vectored IRQs have the lowest priority.
 *
 *   The general flow of IRQ processing is to simply read the VICAddress
 *   and jump to the address of the vector provided in the register.  The
 *   VIC will provide the address of the highest priority vectored IRQ.
 *   If a non-vectored IRQ is requesting, the address of a default handler
 *   is provided.
 *
 ****************************************************************************/

#ifndef CONFIG_VECTORED_INTERRUPTS
uint32_t *arm_decodeirq(uint32_t *regs)
#else
static uint32_t *lpc23xx_decodeirq(uint32_t *regs)
#endif
{
  struct tcb_s *tcb = this_task();

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  err("ERROR: Unexpected IRQ\n");

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);
  PANIC();
  return NULL;
#else

  /* Check which IRQ fires */

  uint32_t irqbits = vic_getreg(VIC_IRQSTATUS_OFFSET) & 0xffffffff;
  unsigned int irq;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      if (irqbits & (uint32_t) (1 << irq))
        {
          break;
        }
    }

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)            /* redundant check ?? */
    {
      uint32_t *saveregs;
      bool savestate;

      savestate = up_interrupt_context();
      saveregs = tcb->xcp.regs;
      up_set_interrupt_context(true);
      tcb->xcp.regs = regs;

      /* Acknowledge the interrupt */

      arm_ack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of saveregs. */

      up_set_interrupt_context(savestate);
      tcb->xcp.regs = saveregs;
    }

  return NULL;  /* Return not used in this architecture */
#endif
}

#ifdef CONFIG_VECTORED_INTERRUPTS
uint32_t *arm_decodeirq(uint32_t *regs)
{
  vic_vector_t vector = (vic_vector_t) vic_getreg(VIC_ADDRESS_OFFSET);

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  /* Valid Interrupt */

  if (vector != NULL)
    {
      (vector)(regs);
    }

  return NULL;  /* Return not used in this architecture */
}
#endif
