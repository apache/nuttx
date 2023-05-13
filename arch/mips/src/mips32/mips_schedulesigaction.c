/****************************************************************************
 * arch/mips/src/mips32/mips_schedulesigaction.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/mips32/cp0.h>

#include "sched/sched.h"
#include "mips_internal.h"

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

void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
  uint32_t status;

  sinfo("tcb=%p sigdeliver=%p\n", tcb, sigdeliver);

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      tcb->xcp.sigdeliver = sigdeliver;

      /* First, handle some special cases when the signal is
       * being delivered to the currently executing task.
       */

      sinfo("rtcb=%p CURRENT_REGS=%p\n",
            this_task(), CURRENT_REGS);

      if (tcb == this_task())
        {
          /* CASE 1:  We are not in an interrupt handler and
           * a task is signalling itself for some reason.
           */

          if (!CURRENT_REGS)
            {
              /* In this case just deliver the signal now. */

              sigdeliver(tcb);
              tcb->xcp.sigdeliver = NULL;
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
              /* Save the return EPC and STATUS registers.  These will be
               * restored by the signal trampoline after the signals have
               * been delivered.
               */

              tcb->xcp.saved_epc        = CURRENT_REGS[REG_EPC];
              tcb->xcp.saved_status     = CURRENT_REGS[REG_STATUS];

              /* Then set up to vector to the trampoline with interrupts
               * disabled
               */

              CURRENT_REGS[REG_EPC]     = (uint32_t)mips_sigdeliver;
              status                    = CURRENT_REGS[REG_STATUS];
              status                   &= ~CP0_STATUS_INT_MASK;
              status                   |= CP0_STATUS_INT_SW0;
              CURRENT_REGS[REG_STATUS]  = status;

              /* And make sure that the saved context in the TCB
               * is the same as the interrupt return context.
               */

              mips_savestate(tcb->xcp.regs);

              sinfo("PC/STATUS Saved: %08" PRIx32 "/%08" PRIx32
                    " New: %08" PRIx32 "/%08" PRIx32 "\n",
                    tcb->xcp.saved_epc, tcb->xcp.saved_status,
                    CURRENT_REGS[REG_EPC], CURRENT_REGS[REG_STATUS]);
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

          tcb->xcp.saved_epc         = tcb->xcp.regs[REG_EPC];
          tcb->xcp.saved_status      = tcb->xcp.regs[REG_STATUS];

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          tcb->xcp.regs[REG_EPC]     = (uint32_t)mips_sigdeliver;
          status                     = tcb->xcp.regs[REG_STATUS];
          status                    &= ~CP0_STATUS_INT_MASK;
          status                    |= CP0_STATUS_INT_SW0;
          tcb->xcp.regs[REG_STATUS]  = status;

          sinfo("PC/STATUS Saved: %08" PRIx32 "/%08" PRIx32
                " New: %08" PRIx32 "/%08" PRIx32 "\n",
                tcb->xcp.saved_epc, tcb->xcp.saved_status,
                tcb->xcp.regs[REG_EPC], tcb->xcp.regs[REG_STATUS]);
        }
    }
}
