/****************************************************************************
 * arch/xtensa/src/common/xtensa_sigdeliver.c
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
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_sig_deliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void xtensa_sig_deliver(void)
{
  struct tcb_s *rtcb = this_task();
  uint32_t regs[XCPTCONTEXT_REGS];

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = get_errno();

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the return state on the stack. */

  xtensa_copystate(regs, rtcb->xcp.regs);

#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount - 1;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need to call leave_critical_section() repeatedly to get the
   * irqcount to zero, freeing all global spinlocks that enforce the critical
   * section.
   */

  do
    {
      leave_critical_section((regs[REG_PS]));
    }
  while (rtcb->irqcount > 0);
#endif /* CONFIG_SMP */

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

  /* Call enter_critical_section() to disable local interrupts before
   * restoring local context.
   *
   * Here, we should not use up_irq_save() in SMP mode.
   * For example, if we call up_irq_save() here and another CPU might
   * have called up_cpu_pause() to this cpu, hence g_cpu_irqlock has
   * been locked by the cpu, in this case, we would see a deadlock in
   * later call of enter_critical_section() to restore irqcount.
   * To avoid this situation, we need to call enter_critical_section().
   */

#ifdef CONFIG_SMP
  enter_critical_section();
#else
  up_irq_save();
#endif

  /* Restore the saved errno value */

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

  regs[REG_PC]         = rtcb->xcp.saved_pc;
  regs[REG_PS]         = rtcb->xcp.saved_ps;
  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Issue:
   *
   * Task1 --> process
   *       --> xtensa_context_save(S1)
   *       --> s32i a0, a2, (4 * REG_A0)
   *       --> rtcb->xcp.regs[REG_A0] = A0
   *
   * Task preemption
   *
   * Task2 --> Post signal to Task1
   *       --> Wake up Task1
   *
   * Task1 --> xtensa_sig_deliver
   *       --> up_irq_enable()
   *       --> Task preemption
   *
   * Task preemption --> xtensa_context_save
   *                 --> rtcb->xcp.regs[REG_A0] = A0 of "xtensa_sig_deliver"
   *                     = _xtensa_sig_trampoline + 6
   *                     = "j 1b"
   *
   * Process ...
   *
   * Task1 --> xtensa_sig_deliver
   *       --> xtensa_context_restore
   *       --> xtensa_context_save(S1)
   *       --> l32i a0, a2, (4 * REG_A0)
   *       --> a0 = "j 1b"
   *       --> ret
   *       --> run "j 1b"
   */

  rtcb->xcp.regs[REG_A0] = regs[REG_A0];

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   *
   * REVISIT:  irqcount should be one from the above call to
   * enter_critical_section().  Could the saved_irqcount be zero?  That
   * would be a problem.
   */

  DEBUGASSERT(rtcb->irqcount == 1);
  while (rtcb->irqcount < saved_irqcount)
    {
      enter_critical_section();
    }
#endif

  /* Then restore the correct state for this thread of execution.
   * NOTE: The co-processor state should already be correct.
   */

  board_autoled_off(LED_SIGNAL);
  xtensa_context_restore(regs);
}
