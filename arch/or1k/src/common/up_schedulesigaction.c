/****************************************************************************
 * arch/or1k/src/common/up_schedulesigaction.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'sigdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
  irqstate_t flags;

  sinfo("tcb=0x%p sigdeliver=0x%p\n", tcb, sigdeliver);

  /* Make sure that interrupts are disabled */

  flags = enter_critical_section();

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      /* First, handle some special cases when the signal is
       * being delivered to the currently executing task.
       */

      sinfo("rtcb=0x%p CURRENT_REGS=0x%p\n", this_task(), CURRENT_REGS);

      if (tcb == this_task())
        {
          /* CASE 1:  We are not in an interrupt handler and
           * a task is signalling itself for some reason.
           */

          if (!CURRENT_REGS)
            {
              /* In this case just deliver the signal now. */

              sigdeliver(tcb);
            }

          /* CASE 2:  We are in an interrupt handler AND the
           * interrupted task is the same as the one that
           * must receive the signal, then we will have to modify
           * the return state as well as the state in the TCB.
           *
           * Hmmm... there looks like a latent bug here: The following
           * logic would fail in the strange case where we are in an
           * interrupt handler, the thread is signalling itself, but
           * a context switch to another task has occurred so that
           * CURRENT_REGS does not refer to the thread of this_task()!
           */

          else
            {
              /* Save the return lr and cpsr and one scratch register
               * These will be restored by the signal trampoline after
               * the signals have been delivered.
               */

              tcb->xcp.sigdeliver       = sigdeliver;
              /* tcb->xcp.saved_pc         = CURRENT_REGS[REG_PC];
               * tcb->xcp.saved_cpsr       = CURRENT_REGS[REG_CPSR];
               */

              /* Then set up to vector to the trampoline with interrupts
               * disabled
               */

              /* CURRENT_REGS[REG_PC]      = (uint32_t)up_sigdeliver;
               * CURRENT_REGS[REG_CPSR]    = SVC_MODE | PSR_I_BIT |
               *                             PSR_F_BIT;
               */

              /* And make sure that the saved context in the TCB
               * is the same as the interrupt return context.
               */

              up_savestate(tcb->xcp.regs);
            }
        }

      /* Otherwise, we are (1) signaling a task is not running
       * from an interrupt handler or (2) we are not in an
       * interrupt handler and the running task is signalling
       * some non-running task.
       */

      else
        {
          /* Save the return lr and cpsr and one scratch register
           * These will be restored by the signal trampoline after
           * the signals have been delivered.
           */

          tcb->xcp.sigdeliver       = sigdeliver;
          tcb->xcp.saved_pc         = tcb->xcp.regs[REG_PC];

          /* tcb->xcp.saved_cpsr       = tcb->xcp.regs[REG_CPSR]; */

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          /* tcb->xcp.regs[REG_PC]      = (uint32_t)up_sigdeliver;
           * tcb->xcp.regs[REG_CPSR]    = SVC_MODE | PSR_I_BIT | PSR_F_BIT;
           */
        }
    }

  leave_critical_section(flags);
}
