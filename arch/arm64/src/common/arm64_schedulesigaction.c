/****************************************************************************
 * arch/arm64/src/common/arm64_schedulesigaction.c
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

#include "sched/sched.h"
#include "signal/signal.h"
#include "arm64_internal.h"
#include "arm64_arch.h"
#include "irq/irq.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void arm64_init_signal_process(struct tcb_s *tcb, uint64_t *regs)
{
/****************************************************************************
 * if regs != NULL We are interrupting the context,
 * we should modify the regs
 ****************************************************************************/

  regs = (regs != NULL) ? regs : tcb->xcp.regs;

  tcb->xcp.regs = (uint64_t *)(regs[REG_SP_ELX] - XCPTCONTEXT_SIZE * 2);
  memset(tcb->xcp.regs, 0, XCPTCONTEXT_SIZE);

  tcb->xcp.regs[REG_ELR]    = (uint64_t)arm64_sigdeliver;

  /* Keep using SP_EL1 */

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
  tcb->xcp.regs[REG_SPSR]   = SPSR_MODE_EL3H | DAIF_FIQ_BIT | DAIF_IRQ_BIT;
#else
  tcb->xcp.regs[REG_SPSR]   = SPSR_MODE_EL1H | DAIF_FIQ_BIT | DAIF_IRQ_BIT;
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  tcb->xcp.regs[REG_SP_EL0] = regs[REG_SP_EL0];
#else
  tcb->xcp.regs[REG_SP_EL0] = regs[REG_SP_ELX] - XCPTCONTEXT_SIZE * 2;
#endif
  tcb->xcp.regs[REG_SP_ELX] = regs[REG_SP_ELX] - XCPTCONTEXT_SIZE;
  tcb->xcp.regs[REG_EXE_DEPTH]  = 1;
}

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
       * REVISIT:  Signal handler will run in a critical section!
       */

      nxsig_deliver(tcb);
      tcb->flags &= ~TCB_FLAG_SIGDELIVER;
    }
  else
    {
      /* Save the return lr and cpsr and one scratch register.  These
       * will be restored by the signal trampoline after the signals
       * have been delivered.
       */

      tcb->xcp.saved_regs = tcb->xcp.regs;

      /* create signal process context */

      arm64_init_signal_process(tcb, NULL);
    }
}
