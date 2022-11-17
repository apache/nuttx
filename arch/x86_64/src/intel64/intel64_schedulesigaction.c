/****************************************************************************
 * arch/x86_64/src/intel64/intel64_schedulesigaction.c
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

#include "sched/sched.h"
#include "x86_64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more signal handling
 *   actions have been queued for execution.  The architecture specific code
 *   must configure things so that the 'sigdeliver' callback is executed on
 *   the thread specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked nor should it
 *   cause any immediate execution of sigdeliver. Typically, a few cases need
 *   to be considered:
 *
 *   (1) This function may be called from an interrupt handler. During
 *       interrupt processing, all xcptcontext structures should be valid for
 *       all tasks.  That structure should be modified to invoke sigdeliver()
 *       either on return from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT the currently
 *       executing task, then again just modify the saved xcptcontext
 *       structure for the recipient task so it will invoke sigdeliver when
 *       that task is later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the currently
 *       executing task -- just call the signal handler now.
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
  sinfo("tcb=%p sigdeliver=%p\n", tcb, sigdeliver);
  sinfo("rtcb=%p g_current_regs=%p\n", this_task(), g_current_regs);

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      /* First, handle some special cases when the signal is being delivered
       * to the currently executing task.
       */

      if (tcb == this_task())
        {
          /* CASE 1:  We are not in an interrupt handler and a task is
           * signalling itself for some reason.
           */

          if (!g_current_regs)
            {
              /* In this case just deliver the signal with a function call
               * now.
               */

              sigdeliver(tcb);
            }

          /* CASE 2:  We are in an interrupt handler AND the interrupted task
           * is the same as the one that must receive the signal, then we
           * will have to modify the return state as well as the state in the
           * TCB.
           *
           * Hmmm... there looks like a latent bug here: The following logic
           * would fail in the strange case where we are in an interrupt
           * handler, the thread is signalling itself, but a context switch
           * to another task has occurred so that g_current_regs does not
           * refer to the thread of this_task()!
           */

          else
            {
              /* Save the return lr and cpsr and one scratch register. These
               * will be restored by the signal trampoline after the signals
               * have been delivered.
               */

              tcb->xcp.sigdeliver       = sigdeliver;
              tcb->xcp.saved_rip        = g_current_regs[REG_RIP];
              tcb->xcp.saved_rsp        = tcb->xcp.regs[REG_RSP];
              tcb->xcp.saved_rflags     = g_current_regs[REG_RFLAGS];

              /* Then set up to vector to the trampoline with interrupts
               * disabled
               */

              g_current_regs[REG_RIP]     = (uint64_t)x86_64_sigdeliver;
              g_current_regs[REG_RFLAGS]  = 0;

              /* And make sure that the saved context in the TCB
               * is the same as the interrupt return context.
               */

              x86_64_savestate(tcb->xcp.regs);
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
          tcb->xcp.saved_rip        = tcb->xcp.regs[REG_RIP];
          tcb->xcp.saved_rsp        = tcb->xcp.regs[REG_RSP];
          tcb->xcp.saved_rflags     = tcb->xcp.regs[REG_RFLAGS];

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          tcb->xcp.regs[REG_RIP]    = (uint64_t)x86_64_sigdeliver;
          tcb->xcp.regs[REG_RFLAGS]  = 0;
        }
    }
}
