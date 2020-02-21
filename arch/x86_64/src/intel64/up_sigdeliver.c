/****************************************************************************
 * arch/x86_64/src/intel64/up_sigdeliver.c
 *
 *   Copyright (C) 2011, 2015 Gregory Nutt. All rights reserved.
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
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
  struct tcb_s *rtcb = this_task();
  uint64_t regs_area[XCPTCONTEXT_REGS + 2];
  uint64_t* regs;
  regs = (uint64_t*)(((uint64_t)(regs_area) + 15) & (~(uint64_t)15)); // align regs to 16byte boundary for SSE instrucitons
  sig_deliver_t sigdeliver;

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = rtcb->pterrno;

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  ASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the real return state on the stack. */

  up_copystate(regs, rtcb->xcp.regs);
  regs[REG_RIP]        = rtcb->xcp.saved_rip;
  regs[REG_RFLAGS]     = rtcb->xcp.saved_rflags;

  /* Get a local copy of the sigdeliver function pointer. we do this so that
   * we can nullify the sigdeliver function pointer in the TCB and accept
   * more signal deliveries while processing the current pending signals.
   */

  sigdeliver           = rtcb->xcp.sigdeliver;
  rtcb->xcp.sigdeliver = NULL;

  /* Then restore the task interrupt state */

  up_irq_restore(regs[REG_RFLAGS]);

  /* Deliver the signals */

  sigdeliver(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");
  (void)up_irq_save();
  rtcb->pterrno = saved_errno;

  if(rtcb->xcp.saved_rsp){
    regs[REG_RSP]      = rtcb->xcp.saved_rsp;
    rtcb->xcp.saved_rsp = 0;
  }

  /* Then restore the correct state for this thread of execution. */

  board_autoled_off(LED_SIGNAL);
  up_fullcontextrestore(regs);
}

#endif /* !CONFIG_DISABLE_SIGNALS */

