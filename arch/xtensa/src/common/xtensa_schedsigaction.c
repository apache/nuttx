/****************************************************************************
 * arch/xtensa/src/common/xtensa_schedsigaction.c
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

#include "irq/irq.h"
#include "sched/sched.h"

#include "chip.h"
#include "xtensa.h"

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

          /* Save the context registers.  These will be restored by the
           * signal trampoline after the signals have been delivered.
           *
           * NOTE: that hi-priority interrupts are not disabled.
           */

          tcb->xcp.saved_regs   = tcb->xcp.regs;

          if ((tcb->xcp.saved_regs[REG_PS] & PS_EXCM_MASK) != 0)
            {
              tcb->xcp.saved_regs[REG_PS] &= ~PS_EXCM_MASK;
            }

          /* Duplicate the register context.  These will be
           * restored by the signal trampoline after the signal has been
           * delivered.
           */

          tcb->xcp.regs         = (void *)
                                  ((uint32_t)tcb->xcp.regs -
                                             XCPTCONTEXT_SIZE);
          memcpy(tcb->xcp.regs, tcb->xcp.saved_regs, XCPTCONTEXT_SIZE);

          tcb->xcp.regs[REG_A1] = (uint32_t)tcb->xcp.regs +
                                            XCPTCONTEXT_SIZE;

          /* Then set up to vector to the trampoline with interrupts
           * disabled
           */

          tcb->xcp.regs[REG_PC] = (uint32_t)xtensa_sig_deliver;
#ifdef __XTENSA_CALL0_ABI__
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
          tcb->xcp.regs[REG_PS] = (uint32_t)
              (PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM |
               PS_WOE | PS_CALLINC(1));
#endif
#ifndef CONFIG_BUILD_FLAT
          xtensa_raiseprivilege(tcb->xcp.regs);
#endif

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
