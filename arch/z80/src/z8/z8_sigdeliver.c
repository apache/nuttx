/****************************************************************************
 * arch/z80/src/z8/z8_sigdeliver.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "chip/switch.h"
#include "sched/sched.h"
#include "z80_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_copystate
 ****************************************************************************/

/* Maybe a little faster than most memcpy's */

static void z8_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src)
{
  int i;
  for (i = 0; i < XCPTCONTEXT_REGS; i++)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void z80_sigdeliver(void)
{
  FAR struct tcb_s *rtcb = this_task();
  chipreg_t regs[XCPTCONTEXT_REGS];

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

  z8_copystate(regs, rtcb->xcp.regs);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signals */

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

  regs[XCPT_PC]        = rtcb->xcp.saved_pc;
  regs[XCPT_IRQCTL]    = rtcb->xcp.saved_irqctl;
  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Then restore the correct state for this thread of execution. */

  board_autoled_off(LED_SIGNAL);
  z8_restorecontext(regs);
}
