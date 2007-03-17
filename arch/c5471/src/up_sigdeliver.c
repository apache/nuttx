/************************************************************
 * up_sigdeliver.c
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sched.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include "os_internal.h"
#include "up_internal.h"
#include "c5471.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Funtions
 ************************************************************/

/************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a
 *   signal action was posted.  The task context was mucked
 *   with and forced to branch to this location with interrupts
 *   disabled.
 *
 ************************************************************/

void up_sigdeliver(void)
{
  _TCB  *rtcb = (_TCB*)g_readytorun.head;
  uint32 regs[XCPTCONTEXT_REGS];
  sig_deliver_t sigdeliver;

  up_ledon(LED_SIGNAL);

  dbg("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
       rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  ASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the real return state on the stack. */

  up_copystate(regs, rtcb->xcp.regs);
  regs[REG_PC]   = rtcb->xcp.saved_pc;
  regs[REG_CPSR] = rtcb->xcp.saved_cpsr;

  /* Get a local copy of the sigdeliver function pointer.
   * we do this so that we can nullify the sigdeliver
   * function point in the TCB and accept more signal
   * deliveries while processing the current pending
   * signals.
   */

  sigdeliver           = rtcb->xcp.sigdeliver;
  rtcb->xcp.sigdeliver = NULL;

  /* Then restore the task interrupt statat. */

  irqrestore(regs[REG_CPSR]);

  /* Deliver the signals */

  sigdeliver(rtcb);

  /* Then restore the correct state for this thread of
   * execution.
   */

  up_ledoff(LED_SIGNAL);
  dbg("Resuming\n");
  up_fullcontextrestore(regs);
}
