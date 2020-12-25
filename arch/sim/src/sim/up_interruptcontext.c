/****************************************************************************
 * arch/sim/src/sim/up_interruptcontext.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <nuttx/arch.h>

#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt handler
 *   context.
 *
 ****************************************************************************/

bool up_interrupt_context(void)
{
  return CURRENT_REGS != NULL;
}

/****************************************************************************
 * Name: up_doirq
 ****************************************************************************/

void *up_doirq(int irq, void *context)
{
  /* Allocate temporary context on the stack */

  xcpt_reg_t tmp[XCPTCONTEXT_REGS];
  void *regs = (void *)tmp;

  /* CURRENT_REGS non-zero indicates that we are processing an interrupt.
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

#ifdef CONFIG_SMP
  if (up_setjmp(regs) == 0)
    {
#endif

      CURRENT_REGS = regs;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* If a context switch occurred while processing the interrupt then
       * CURRENT_REGS may have change value.  If we return any value
       * different from the input regs, then the lower level will know that
       * context switch occurred during interrupt processing.
       */

      regs = (void *)CURRENT_REGS;

      /* Restore the previous value of CURRENT_REGS.  NULL would indicate
       * that we are no longer in an interrupt handler.  It will be non-NULL
       * if we are returning from a nested interrupt.
       */

      CURRENT_REGS = NULL;

#ifdef CONFIG_SMP
      /* Handle signal */

      sim_sigdeliver();

      /* Then switch contexts */

      up_longjmp(regs, 1);
    }
#endif

  return regs;
}
