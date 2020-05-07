/****************************************************************************
 * arch/avr/src/avr32/up_sigdeliver.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"

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
#if 0
  uint32_t regs[XCPTCONTEXT_REGS + 3];  /* Why +3? See below */
#else
  uint32_t regs[XCPTCONTEXT_REGS];
#endif

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = get_errno();

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the return state on the stack. */

  up_copystate(regs, rtcb->xcp.regs);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signal */

  ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");
  up_irq_save();
  set_errno(saved_errno);

  /* Modify the saved return state with the actual saved values in the
   * TCB.  This depends on the fact that nested signal handling is
   * not supported.  Therefore, these values will persist throughout the
   * signal handling action.
   *
   * Keeping this data in the TCB resolves a security problem in protected
   * and kernel mode:  The regs[] array is visible on the user stack and
   * could be modified by a hostile program.
   */

  regs[REG_PC]         = rtcb->xcp.saved_pc;
  regs[REG_SR]         = rtcb->xcp.saved_sr;
  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Then restore the correct state for this thread of execution. This is an
   * unusual case that must be handled by up_fullcontextresore. This case is
   * unusual in two ways:
   *
   *   1. It is not a context switch between threads.  Rather,
   *      up_fullcontextrestore must behave more it more like a longjmp
   *      within the same task, using the same stack.
   *   2. In this case, up_fullcontextrestore is called with r12 pointing to
   *      a register save area on the stack to be destroyed.  This is
   *      dangerous because there is the very real possibility that the new
   *      stack pointer might overlap with the register save area and that
   *      stack usage in up_fullcontextrestore might corrupt the register
   *      save data before the state is restored.  At present, there does
   *      not appear to be any stack overlap problems.  If there were, then
   *      adding 3 words to the size of register save structure size will
   *      protect its contents.
   */

  board_autoled_off(LED_SIGNAL);
  up_fullcontextrestore(regs);
}
