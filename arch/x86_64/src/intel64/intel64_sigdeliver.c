/****************************************************************************
 * arch/x86_64/src/intel64/intel64_sigdeliver.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "x86_64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void x86_64_sigdeliver(void)
{
  struct tcb_s *rtcb = this_task();
  uint64_t regs_area[XCPTCONTEXT_REGS + 8];
  uint64_t *regs;

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->sigpendactionq.head);
  DEBUGASSERT((rtcb->flags & TCB_FLAG_SIGDELIVER) != 0);

  /* Align regs to 64 byte boundary for XSAVE */

  regs = (uint64_t *)(((uint64_t)(regs_area) + 63) & (~(uint64_t)63));

  /* Save the real return state on the stack ASAP before any chance we went
   * sleeping and break the register profile.  We entered this function with
   * interrupt disabled, therefore we don't have to worried being preempted
   * by interrupt.
   */

  x86_64_copystate(regs, rtcb->xcp.regs);

#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  while (rtcb->irqcount > 0)
    {
      leave_critical_section((uint8_t)regs[REG_RFLAGS]);
    }
#endif /* CONFIG_SMP */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signals */

  nxsig_deliver(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");

#ifdef CONFIG_SMP
  enter_critical_section();
#else
  up_irq_save();
#endif

  /* Modify the saved return state with the actual saved values in the
   * TCB.  This depends on the fact that nested signal handling is
   * not supported.  Therefore, these values will persist throughout the
   * signal handling action.
   *
   * Keeping this data in the TCB resolves a security problem in protected
   * and kernel mode:  The regs[] array is visible on the user stack and
   * could be modified by a hostile program.
   */

  regs[REG_RIP]    = rtcb->xcp.saved_rip;
  regs[REG_RSP]    = rtcb->xcp.saved_rsp;
  regs[REG_RFLAGS] = rtcb->xcp.saved_rflags;

  /* Allows next handler to be scheduled */

  rtcb->flags &= ~TCB_FLAG_SIGDELIVER;

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   *
   * REVISIT:  irqcount should be one from the above call to
   * enter_critical_section().  Could the saved_irqcount be zero?  That
   * would be a problem.
   */

  DEBUGASSERT(rtcb->irqcount == 1);
  while (rtcb->irqcount < saved_irqcount)
    {
      enter_critical_section();
    }
#endif

  /* Then restore the correct state for this thread of execution. */

  board_autoled_off(LED_SIGNAL);
  x86_64_fullcontextrestore(regs);
}
