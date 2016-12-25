/****************************************************************************
 * arch/xtensa/src/common/xtena_irqdispatch.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <assert.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/chip/core-isa.h>

#include "xtensa.h"

#include "group/group.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *xtensa_irq_dispatch(int irq, uint32_t *regs)
{
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  board_autoled_on(LED_INIRQ);
  PANIC();

#else
#if XCHAL_CP_NUM > 0
  /* Save the TCB of in case we need to save co-processor state */

  struct tcb_s *tcb = this_task();
#endif

  board_autoled_on(LED_INIRQ);

  /* Nested interrupts are not supported */

  DEBUGASSERT(CURRENT_REGS == NULL);

  /* Current regs non-zero indicates that we are processing an interrupt;
   * CURRENT_REGS is also used to manage interrupt level context switches.
   */

  CURRENT_REGS = regs;

  /* Deliver the IRQ
   *
   * NOTE: Co-process state has not been saved yet (see below).  As a
   * consequence, no interrupt level logic may perform co-processor
   * operations.  This includes use of the FPU.
   */

  irq_dispatch(irq, regs);

#if XCHAL_CP_NUM > 0 || defined(CONFIG_ARCH_ADDRENV)
  /* Check for a context switch.  If a context switch occurred, then
   * CURRENT_REGS will have a different value than it did on entry.
   */

  if (regs != CURRENT_REGS)
    {
#if XCHAL_CP_NUM > 0
       /* If an interrupt level context switch has occurred, then save the
        * co-processor state in in the suspended thread's co-processor save
        * area.
        *
        * NOTE 1. The state of the co-processor has not been altered and
        *         still represents the to-be-suspended thread.
        * NOTE 2. We saved a reference  TCB of the original thread on entry.
        */

       xtensa_coproc_savestate(&tcb->xcp.cpstate);

       /* Then set up the co-processor state for the to-be-started thread.
        *
        * NOTE: The current thread for this CPU is the to-be-started
        * thread.
        */

       tcb = this_task();
       xtensa_coproc_restorestate(&tcb->xcp.cpstate);
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

  /* Set CURRENT_REGS to NULL to indicate that we are no longer in an
   * interrupt handler.
   */

  regs         = (uint32_t *)CURRENT_REGS;
  CURRENT_REGS = NULL;
#endif

  board_autoled_off(LED_INIRQ);
  return regs;
}
