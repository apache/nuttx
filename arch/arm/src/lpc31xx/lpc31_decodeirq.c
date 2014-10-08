/********************************************************************************
 * arch/arm/src/lpc31xx/lpc31_decodeirq.c
 *
 *   Copyright (C) 2009, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include "up_arch.h"

#include "up_internal.h"
#include "group/group.h"

#include "lpc31_intc.h"

/********************************************************************************
 * Pre-processor Definitions
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
 * Public Functions
 ********************************************************************************/

void up_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  lowsyslog(LOG_ERR, "Unexpected IRQ\n");
  current_regs = regs;
  PANIC();
#else
  int index;
  int irq;

  /* Read the IRQ vector status register.  Bits 3-10 provide the IRQ number
   * of the interrupt (the TABLE_ADDR was initialized to zero, so the
   * following masking should be unnecessary)
   */

  index = getreg32(LPC31_INTC_VECTOR0) & INTC_VECTOR_INDEX_MASK;
  if (index != 0)
    {
      /* Shift the index so that the range of IRQ numbers are in bits 0-7 (values
       * 1-127) and back off the IRQ number by 1 so that the numbering is zero-based
       */

      irq = (index >> INTC_VECTOR_INDEX_SHIFT) -1;

      /* Verify that the resulting IRQ number is valid */

      if ((unsigned)irq < NR_IRQS)
        {
          /* Mask and acknowledge the interrupt */

          up_maskack_irq(irq);

          /* Current regs non-zero indicates that we are processing an interrupt;
           * current_regs is also used to manage interrupt level context switches.
           *
           * Nested interrupts are not supported.
           */

          DEBUGASSERT(current_regs == NULL);
          current_regs = regs;

          /* Deliver the IRQ */

          irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
          /* Check for a context switch.  If a context switch occurred, then
           * current_regs will have a different value than it did on entry.
           * If an interrupt level context switch has occurred, then restore
           * the floating point state and the establish the correct address
           * environment before returning from the interrupt.
           */

          if (regs != current_regs)
            {
#ifdef CONFIG_ARCH_FPU
              /* Restore floating point registers */

              up_restorefpu((uint32_t*)current_regs);
#endif

#ifdef CONFIG_ARCH_ADDRENV
              /* Make sure that the address environment for the previously
               * running task is closed down gracefully (data caches dump,
               * MMU flushed) and set up the address environment for the new
               * thread at the head of the ready-to-run list.
               */

              (void)group_addrenv(NULL);
#endif
            }
#endif
          /* Set current_regs to NULL to indicate that we are no longer in an
           * interrupt handler.
           */

          current_regs = NULL;

          /* Unmask the last interrupt (global interrupts are still
           * disabled).
           */

          up_enable_irq(irq);
        }
    }
#endif
}
