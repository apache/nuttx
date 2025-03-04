/****************************************************************************
 * arch/tricore/src/common/tricore_schedulesigaction.c
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

#include <inttypes.h>
#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "tricore_internal.h"

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
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

void up_schedule_sigaction(struct tcb_s *tcb)
{
  /* First, handle some special cases when the signal is
   * being delivered to the currently executing task.
   */

  if (tcb == this_task())
    {
      /* CASE 1:  We are not in an interrupt handler and
       * a task is signalling itself for some reason.
       */

      if (up_current_regs() == NULL)
        {
          /* In this case just deliver the signal now. */

          nxsig_deliver(tcb);
          tcb->flags &= ~TCB_FLAG_SIGDELIVER;
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
       * g_current_regs does not refer to the thread of this_task()!
       */

      else
        {
          /* Save the context registers.  These will be
           * restored by the signal trampoline after the signals have
           * been delivered.
           */

          tricore_savestate(tcb->xcp.saved_regs);

          /* Create a new CSA for signal delivery. The new context
           * will borrow the process stack of the current tcb.
           */

          up_set_current_regs(tricore_alloc_csa((uintptr_t)
            tricore_sigdeliver,
            STACK_ALIGN_DOWN(up_getusrsp(tcb->xcp.regs)),
            PSW_IO_SUPERVISOR | PSW_CDE, true));
        }
    }

  /* Otherwise, we are (1) signaling a task is not running
   * from an interrupt handler or (2) we are not in an
   * interrupt handler and the running task is signalling
   * some non-running task.
   */

  else
    {
      /* Save the return EPC and STATUS registers.  These will be
       * restored by the signal trampoline after the signals have
       * been delivered.
       */

      /* Save the current register context location */

      tcb->xcp.saved_regs = tcb->xcp.regs;

      /* Create a new CSA for signal delivery. The new context
       * will borrow the process stack of the current tcb.
       */

      tcb->xcp.regs = tricore_alloc_csa((uintptr_t)tricore_sigdeliver,
        STACK_ALIGN_DOWN(up_getusrsp(tcb->xcp.regs)),
        PSW_IO_SUPERVISOR | PSW_CDE, true);
    }
}
