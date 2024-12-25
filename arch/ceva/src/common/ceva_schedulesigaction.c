/****************************************************************************
 * arch/ceva/src/common/ceva_schedulesigaction.c
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

#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "ceva_internal.h"

#ifndef CONFIG_DISABLE_SIGNALS

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

void up_schedule_sigaction(struct tcb_s *tcb)
{
  sinfo("tcb=%p, rtcb=%p current_regs=%p\n", tcb,
        this_task(), up_current_regs());

  /* First, handle some special cases when the signal is being delivered
   * to task that is currently executing on any CPU.
   */

  if (tcb->task_state == TSTATE_TASK_RUNNING)
    {
      uint8_t me  = this_cpu();
#ifdef CONFIG_SMP
      uint8_t cpu = tcb->cpu;
#else
      uint8_t cpu = 0;
#endif

      /* CASE 1:  We are not in an interrupt handler and a task is
       * signaling itself for some reason.
       */

      if (cpu == me && !up_current_regs())
        {
          /* In this case just deliver the signal now. */

          nxsig_deliver(tcb);
          tcb->flags &= ~TCB_FLAG_SIGDELIVER;
        }

      /* CASE 2:  The task that needs to receive the signal is running.
       * This could happen if the task is running on another CPU OR if
       * we are in an interrupt handler and the task is running on this
       * CPU.  In the former case, we will have to PAUSE the other CPU
       * first.  But in either case, we will have to modify the return
       * state as well as the state in the TCB.
       */

      else
        {
          /* Save the current register context location */

          tcb->xcp.saved_regs = up_current_regs();

          /* Duplicate the register context.  These will be
           * restored by the signal trampoline after the signal has been
           * delivered.
           */

          up_current_regs() -= XCPTCONTEXT_REGS;
          memcpy(up_current_regs(), up_current_regs() +
                  XCPTCONTEXT_REGS, XCPTCONTEXT_SIZE);

          up_current_regs()[REG_SP]  = (uint32_t)up_current_regs();

          /* Then set up to vector to the trampoline with interrupts
           * unchanged.  We must already be in privileged thread mode
           * to be here.
           */

          up_current_regs()[REG_PC]  = (uint32_t)ceva_sigdeliver;
#ifdef REG_OM
          up_current_regs()[REG_OM] &= ~REG_OM_MASK;
          up_current_regs()[REG_OM] |=  REG_OM_KERNEL;
#endif
        }
    }

  /* Otherwise, we are (1) signaling a task is not running from an
   * interrupt handler or (2) we are not in an interrupt handler and the
   * running task is signaling some other non-running task.
   */

  else
    {
      /* Save the current register context location */

      tcb->xcp.saved_regs = tcb->xcp.regs;

      /* Duplicate the register context.  These will be restored
       * by the signal trampoline after the signal has been delivered.
       */

      tcb->xcp.regs -= XCPTCONTEXT_REGS;
      memcpy(tcb->xcp.regs, tcb->xcp.regs +
             XCPTCONTEXT_REGS, XCPTCONTEXT_SIZE);

      tcb->xcp.regs[REG_SP]  = (uint32_t)tcb->xcp.regs;

      /* Then set up to vector to the trampoline with interrupts
       * unchanged.  We must already be in privileged thread mode to be
       * here.
       */

      tcb->xcp.regs[REG_PC]  = (uint32_t)ceva_sigdeliver;
#ifdef REG_OM
      tcb->xcp.regs[REG_OM] &= ~REG_OM_MASK;
      tcb->xcp.regs[REG_OM] |=  REG_OM_KERNEL;
#endif
    }
}

#endif /* !CONFIG_DISABLE_SIGNALS */
