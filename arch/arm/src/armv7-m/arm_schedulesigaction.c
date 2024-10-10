/****************************************************************************
 * arch/arm/src/armv7-m/arm_schedulesigaction.c
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
#include <arch/armv7-m/nvicpri.h>

#include "psr.h"
#include "exc_return.h"
#include "sched/sched.h"
#include "arm_internal.h"
#include "irq/irq.h"

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
  sinfo("tcb=%p, rtcb=%p current_regs=%p\n", tcb, this_task(),
        this_task()->xcp.regs);

  /* First, handle some special cases when the signal is
   * being delivered to the currently executing task.
   */

  if (tcb == this_task() && !up_interrupt_context())
    {
      /* In this case just deliver the signal now.
       * REVISIT:  Signal handle will run in a critical section!
       */

      (tcb->sigdeliver)(tcb);
      tcb->sigdeliver = NULL;
    }
  else
    {
      /* Save the return PC, CPSR and either the BASEPRI or PRIMASK
       * registers (and perhaps also the LR).  These will be restored
       * by the signal trampoline after the signal has been delivered.
       */

      /* Save the current register context location */

      tcb->xcp.saved_regs           = tcb->xcp.regs;

      /* Duplicate the register context.  These will be
       * restored by the signal trampoline after the signal has been
       * delivered.
       */

      tcb->xcp.regs                 = (void *)
                                      ((uint32_t)tcb->xcp.regs -
                                                 XCPTCONTEXT_SIZE);
      memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

      tcb->xcp.regs[REG_SP]         = (uint32_t)tcb->xcp.regs +
                                                XCPTCONTEXT_SIZE;

      /* Then set up to vector to the trampoline with interrupts
       * disabled.  We must already be in privileged thread mode to be
       * here.
       */

      tcb->xcp.regs[REG_PC]         = (uint32_t)arm_sigdeliver;
#ifdef CONFIG_ARMV7M_USEBASEPRI
      tcb->xcp.regs[REG_BASEPRI]    = NVIC_SYSH_DISABLE_PRIORITY;
#else
      tcb->xcp.regs[REG_PRIMASK]    = 1;
#endif
      tcb->xcp.regs[REG_XPSR]       = ARMV7M_XPSR_T;
#ifdef CONFIG_BUILD_PROTECTED
      tcb->xcp.regs[REG_LR]         = EXC_RETURN_THREAD;
      tcb->xcp.regs[REG_EXC_RETURN] = EXC_RETURN_THREAD;
      tcb->xcp.regs[REG_CONTROL]    = getcontrol() & ~CONTROL_NPRIV;
#endif
    }
}
