/********************************************************************************
 * arch/arm/src/str71x/str71x_decodeirq.c
 *
 *   Copyright (C) 2008-2009, 2011, 2015 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

/********************************************************************************
 * Pre-processor Definitions
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
 * up_decodeirq()
 *
 * Description:
 *   Read the IRQ number from the IVR register.  During initialization, the IVR
 *   register was set to zero.  Each SIR[n] register was programmed to contain
 *   the IRQ number.  At IRQ processing time (when this function run), the IVR
 *   should contain the desired IRQ number.
 *
 ********************************************************************************/

void up_decodeirq(uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  board_autoled_on(LED_INIRQ);
  CURRENT_REGS = regs;
  err("ERROR: Unexpected IRQ\n");
  PANIC();
#else
  unsigned int irq;

  /* Read the IRQ number from the IVR register (Could probably get the same
   * info from CIC register without the setup).
   */

  board_autoled_on(LED_INIRQ);
  irq = getreg32(STR71X_EIC_IVR);

  /* Verify that the resulting IRQ number is valid */

  if (irq < NR_IRQS)
    {
      uint32_t *savestate;

      /* Current regs non-zero indicates that we are processing an interrupt;
       * CURRENT_REGS is also used to manage interrupt level context switches.
       */

      savestate     = (uint32_t *)CURRENT_REGS;
      CURRENT_REGS = regs;

      /* Acknowledge the interrupt */

      up_ack_irq(irq);

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* Restore the previous value of CURRENT_REGS.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
       */

      CURRENT_REGS = savestate;
    }
#ifdef CONFIG_DEBUG_FEATURES
  else
    {
      DEBUGPANIC(); /* Normally never happens */
    }
#endif
  board_autoled_off(LED_INIRQ);
#endif
}
