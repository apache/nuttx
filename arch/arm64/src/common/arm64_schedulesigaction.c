/****************************************************************************
 * arch/arm64/src/common/arm64_schedulesigaction.c
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
#include "arm64_internal.h"
#include "arm64_arch.h"
#include "irq/irq.h"
#include "arm64_fatal.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_init_signal_process(struct tcb_s *tcb, struct regs_context *regs)
{
/****************************************************************************
 * if regs != NULL We are interrupting the context,
 * we should modify the regs
 ****************************************************************************/

  struct regs_context  *pctx = (regs != NULL) ? regs :
  (struct regs_context *)tcb->xcp.regs;
  struct regs_context  *psigctx;
  char *stack_ptr    = (char *)pctx->sp_elx - sizeof(struct regs_context);

#ifdef CONFIG_ARCH_FPU
  struct fpu_reg      *pfpuctx;
  pfpuctx            = STACK_PTR_TO_FRAME(struct fpu_reg, stack_ptr);
  tcb->xcp.fpu_regs  = (uint64_t *)pfpuctx;

  /* set fpu context */

  arm64_init_fpu(tcb);
  stack_ptr          = (char *)pfpuctx;
#endif
  psigctx            = STACK_PTR_TO_FRAME(struct regs_context, stack_ptr);
  memset(psigctx, 0, sizeof(struct regs_context));
  psigctx->elr       = (uint64_t)arm64_sigdeliver;

  /* Keep using SP_EL1 */

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
  psigctx->spsr      = SPSR_MODE_EL3H | DAIF_FIQ_BIT | DAIF_IRQ_BIT;
#else
  psigctx->spsr      = SPSR_MODE_EL1H | DAIF_FIQ_BIT | DAIF_IRQ_BIT;
#endif
  psigctx->sp_elx    = (uint64_t)stack_ptr;
#ifdef CONFIG_ARCH_KERNEL_STACK
  psigctx->sp_el0    = (uint64_t)pctx->sp_el0;
#else
  psigctx->sp_el0    = (uint64_t)psigctx;
#endif
  psigctx->exe_depth = 1;

  tcb->xcp.regs      = (uint64_t *)psigctx;
}

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
  sinfo("tcb=%p sigdeliver=%p\n", tcb, sigdeliver);

  /* Refuse to handle nested signal actions */

  if (!tcb->xcp.sigdeliver)
    {
      tcb->xcp.sigdeliver = sigdeliver;

      /* First, handle some special cases when the signal is being delivered
       * to task that is currently executing on any CPU.
       */

      if (tcb == this_task() && !up_interrupt_context())
        {
          /* In this case just deliver the signal now.
           * REVISIT:  Signal handler will run in a critical section!
           */

          sigdeliver(tcb);
          tcb->xcp.sigdeliver = NULL;
        }
      else
        {
#ifdef CONFIG_SMP
          int cpu = tcb->cpu;
          int me  = this_cpu();

          if (cpu != me)
            {
              /* Pause the CPU */

              up_cpu_pause(cpu);
            }
#endif

          /* Save the return lr and cpsr and one scratch register.  These
           * will be restored by the signal trampoline after the signals
           * have been delivered.
           */

#ifdef CONFIG_ARCH_FPU
          tcb->xcp.saved_fpu_regs = tcb->xcp.fpu_regs;
#endif
          tcb->xcp.saved_reg = tcb->xcp.regs;

          /* create signal process context */

          arm64_init_signal_process(tcb, NULL);

#ifdef CONFIG_SMP
          /* RESUME the other CPU if it was PAUSED */

          if (cpu != me)
            {
              up_cpu_resume(cpu);
            }
#endif
        }
    }
}
