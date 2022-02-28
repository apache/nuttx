/****************************************************************************
 * arch/arm/src/arm/arm_sigdeliver.c
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
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void arm_sigdeliver(void)
{
  struct tcb_s *rtcb = this_task();
  uint32_t *regs = rtcb->xcp.saved_regs;

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

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

  /* Modify the saved return state with the actual saved values in the
   * TCB.  This depends on the fact that nested signal handling is
   * not supported.  Therefore, these values will persist throughout the
   * signal handling action.
   *
   * Keeping this data in the TCB resolves a security problem in protected
   * and kernel mode:  The regs[] array is visible on the user stack and
   * could be modified by a hostile program.
   */

  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Then restore the correct state for this thread of execution. */

  board_autoled_off(LED_SIGNAL);
  arm_fullcontextrestore(regs);
}
