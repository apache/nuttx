/****************************************************************************
 * arch/sparc/src/sparc_v8/sparc_v8_sigdeliver.c
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
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "sparc_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sparc_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void sparc_sigdeliver(void)
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

  enter_critical_section();
#endif

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the return state on the stack. */

  sparc_copystate(regs, rtcb->xcp.regs);

retry:
#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount = rtcb->irqcount;
  DEBUGASSERT(saved_irqcount >= 1);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  do
    {
      leave_critical_section((regs[REG_PSR]));
    }
  while (rtcb->irqcount > 0);
#endif /* CONFIG_SMP */

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

  if (!sq_empty(&rtcb->sigpendactionq) &&
      (rtcb->flags & TCB_FLAG_SIGNAL_ACTION) == 0)
    {
      goto retry;
    }

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
  regs[REG_NPC]        = rtcb->xcp.saved_npc;
  regs[REG_PSR]        = rtcb->xcp.saved_status;
  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

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

  /* Then restore the correct state for this thread of execution. This is an
   * unusual case that must be handled by up_fullcontextresore. This case is
   * unusal in two ways:
   *
   *   1. It is not a context switch between threads.  Rather,
   *      sparc_fullcontextrestore must behave more it more like a longjmp
   *      within the same task, using
   *      he same stack.
   *   2. In this case, sparc_fullcontextrestore is called with r12 pointing
   *      to a register save area on the stack to be destroyed.  This is
   *      dangerous because there is the very real possibility that the new
   *      stack pointer might overlap with the register save area and hat
   *      stack usage in sparc_fullcontextrestore might corrupt the register
   *      save data before the state is restored.  At present, there does
   *      not appear to be any stack overlap problems.  If there were, then
   *      adding 3 words to the size of register save structure size will
   *      protect its contents.
   */

  board_autoled_off(LED_SIGNAL);
#ifdef CONFIG_SMP
  /* We need to keep the IRQ lock until task switching */

  rtcb->irqcount++;
  leave_critical_section((regs[REG_PSR]));
  rtcb->irqcount--;
#endif
  sparc_fullcontextrestore(regs);
}
