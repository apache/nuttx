/********************************************************************************
 * arch/arm/src/imx1/imx_decodeirq.c
 *
 *   Copyright (C) 2009, 2011, 2014 Gregory Nutt. All rights reserved.
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
  CURRENT_REGS = regs;
  err("ERROR: Unexpected IRQ\n");
  PANIC();
#else
  uint32_t regval;
  int irq;

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   *
   * Nested interrupts are not supported.
   */

  DEBUGASSERT(CURRENT_REGS == NULL);
  CURRENT_REGS = regs;

  /* Loop while there are pending interrupts to be processed */

  do
    {
      /* Decode the interrupt.  First, fetch the NIVECSR register. */

      regval = getreg32(IMX_AITC_NIVECSR);

      /* The MS 16 bits of the NIVECSR register contains vector index for the
       * highest pending normal interrupt.
       */

      irq = regval >> AITC_NIVECSR_NIVECTOR_SHIFT;

      /* If irq < 64, then this is the IRQ.  If there is no pending interrupt,
       * then irq will be >= 64 (it will be 0xffff for illegal source).
       */

      if (irq < NR_IRQS)
        {
          /* Deliver the IRQ */

          irq_dispatch(irq, regs);

#if defined(CONFIG_ARCH_FPU) || defined(CONFIG_ARCH_ADDRENV)
          /* Check for a context switch.  If a context switch occurred, then
           * CURRENT_REGS will have a different value than it did on entry.
           * If an interrupt level context switch has occurred, then restore
           * the floating point state and the establish the correct address
           * environment before returning from the interrupt.
           */

          if (regs != CURRENT_REGS)
            {
#ifdef CONFIG_ARCH_FPU
              /* Restore floating point registers */

              up_restorefpu((uint32_t *)CURRENT_REGS);
#endif

#ifdef CONFIG_ARCH_ADDRENV
              /* Make sure that the address environment for the previously
               * running task is closed down gracefully (data caches dump,
               * MMU flushed) and set up the address environment for the new
               * thread at the head of the ready-to-run list.
               */

              group_addrenv(NULL);
#endif
            }
#endif
        }
    }
  while (irq < NR_IRQS);

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in
   * an interrupt handler.
   */

  CURRENT_REGS = NULL;
#endif
}
