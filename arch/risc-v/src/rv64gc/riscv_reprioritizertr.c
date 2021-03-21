/****************************************************************************
 * arch/risc-v/src/rv64gc/riscv_reprioritizertr.c
 *
 *   Copyright (C) 2011, 2013-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <syscall.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_reprioritize_rtr
 *
 * Description:
 *   Called when the priority of a running or
 *   ready-to-run task changes and the reprioritization will
 *   cause a context switch.  Two cases:
 *
 *   1) The priority of the currently running task drops and the next
 *      task in the ready to run list has priority.
 *   2) An idle, ready to run task's priority has been raised above the
 *      the priority of the current, running task and it now has the
 *      priority.
 *
 * Input Parameters:
 *   tcb: The TCB of the task that has been reprioritized
 *   priority: The new task priority
 *
 ****************************************************************************/

void up_reprioritize_rtr(struct tcb_s *tcb, uint8_t priority)
{
  /* Verify that the caller is sane */

  if (tcb->task_state < FIRST_READY_TO_RUN_STATE ||
      tcb->task_state > LAST_READY_TO_RUN_STATE
#if SCHED_PRIORITY_MIN > 0
      || priority < SCHED_PRIORITY_MIN
#endif
#if SCHED_PRIORITY_MAX < UINT8_MAX
      || priority > SCHED_PRIORITY_MAX
#endif
    )
    {
       DEBUGPANIC();
    }
  else
    {
      struct tcb_s *rtcb = this_task();
      bool switch_needed;

      sinfo("TCB=%p PRI=%d\n", tcb, priority);

      /* Remove the tcb task from the ready-to-run list.
       * nxsched_remove_readytorun will return true if we just
       * remove the head of the ready to run list.
       */

      switch_needed = nxsched_remove_readytorun(tcb);

      /* Setup up the new task priority */

      tcb->sched_priority = (uint8_t)priority;

      /* Return the task to the specified blocked task list.
       * nxsched_add_readytorun will return true if the task was
       * added to the new list.  We will need to perform a context
       * switch only if the EXCLUSIVE or of the two calls is non-zero
       * (i.e., one and only one the calls changes the head of the
       * ready-to-run list).
       */

      switch_needed ^= nxsched_add_readytorun(tcb);

      /* Now, perform the context switch if one is needed */

      if (switch_needed)
        {
          /* If we are going to do a context switch, then now is the right
           * time to add any pending tasks back into the ready-to-run list.
           * task list now
           */

          if (g_pendingtasks.head)
            {
              nxsched_merge_pending();
            }

          /* Update scheduler parameters */

          nxsched_suspend_scheduler(rtcb);

          /* Are we in an interrupt handler? */

          if (CURRENT_REGS)
            {
              /* Yes, then we have to do things differently.
               * Just copy the CURRENT_REGS into the OLD rtcb.
               */

               riscv_savestate(rtcb->xcp.regs);

              /* Restore the exception context of the rtcb at the (new) head
               * of the ready-to-run task list.
               */

              rtcb = this_task();

              /* Update scheduler parameters */

              nxsched_resume_scheduler(rtcb);

              /* Then switch contexts.  Any necessary address environment
               * changes will be made when the interrupt returns.
               */

              riscv_restorestate(rtcb->xcp.regs);
            }

          /* No, then we will need to perform the user context switch */

          else
            {
              /* Switch context to the context of the task at the head of the
               * ready to run list.
               */

              struct tcb_s *nexttcb = this_task();

#ifdef CONFIG_ARCH_ADDRENV
              /* Make sure that the address environment for the previously
               * running task is closed down gracefully (data caches dump,
               * MMU flushed) and set up the address environment for the new
               * thread at the head of the ready-to-run list.
               */

              (void)group_addrenv(nexttcb);
#endif
              /* Update scheduler parameters */

              nxsched_resume_scheduler(nexttcb);

              /* Then switch contexts */

              riscv_switchcontext(rtcb->xcp.regs, nexttcb->xcp.regs);

              /* riscv_switchcontext forces a context switch to the task at
               * the head of the ready-to-run list.  It does not 'return' in
               * the normal sense. When it does return, it is because the
               * blocked task is again ready to run and has execution
               * priority.
               */
            }
        }
    }
}
