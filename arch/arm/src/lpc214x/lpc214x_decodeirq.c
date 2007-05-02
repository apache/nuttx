/********************************************************************************
 * lpc214x/lpc214x_decodeirq.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"
#include "lpc214x_vic.h"

/********************************************************************************
 * Definitions
 ********************************************************************************/

/********************************************************************************
 * Private Types
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Funstions
 ********************************************************************************/

/********************************************************************************
 * up_decodeirq() and/or lpc214x_decodeirq()
 *
 * Description:
 *   The vectored interrupt controller (VIC) takes 32 interrupt request inputs
 *   and programmatically assigns them into 3 categories:  FIQ, vectored IRQ,
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
void up_decodeirq(uint32 *regs)
#else
static void lpc214x_decodeirq( uint32 *regs)
#endif
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  lib_lowprintf("Unexpected IRQ\n");
  current_regs = regs;
  PANIC(OSERR_ERREXCEPTION);
#else

  /* Decode the interrupt.  First, fetch the interrupt id register. */

  int irq = 0;
#warning "Need to decode the interrupt here"

  /* Verify that the resulting IRQ number is valie */

  if ((unsigned)irq < NR_IRQS)
    {
      /* Mask and acknowledge the interrupt */

      up_maskack_irq(irq);

      /* Current regs non-zero indicates that we are processing an interrupt;
       * current_regs is also used to manage interrupt level context switches.
       */

      current_regs = regs;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Indicate that we are no long in an interrupt handler */

      current_regs = NULL;

      /* Unmask the last interrupt (global interrupts are still
       * disabled.
       */

      up_enable_irq(irq);
    }
#endif
}

#ifdef CONFIG_VECTORED_INTERRUPTS
void up_decodeirq(uint32 *regs)
{
  vic_vector_t vector = (vic_vector)vic_getreg(LPC214X_VIC_VECTADDR_OFFSET);
  vector(regs);
}
#endif
