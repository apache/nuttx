/****************************************************************************
 *  arch/xtensa/src/common/xtensa_unblocktask.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <arch/chip/core-isa.h>

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 * Input Parameters:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run task, executed.
 *
 ****************************************************************************/

void up_unblock_task(struct tcb_s *tcb)
{
  struct tcb_s *rtcb = this_task();

  /* Verify that the context switch can be performed */

  DEBUGASSERT((tcb->task_state >= FIRST_BLOCKED_STATE) &&
              (tcb->task_state <= LAST_BLOCKED_STATE));

  /* Remove the task from the blocked task list */

  nxsched_remove_blocked(tcb);

  /* Add the task in the correct location in the prioritized
   * ready-to-run task list
   */

  if (nxsched_add_readytorun(tcb))
    {
      /* The currently active task has changed! We need to do
       * a context switch to the new task.
       */

      /* Update scheduler parameters */

      nxsched_suspend_scheduler(rtcb);

      /* Are we in an interrupt handler? */

      if (CURRENT_REGS)
        {
          /* Yes, then we have to do things differently.
           * Just copy the CURRENT_REGS into the OLD rtcb.
           */

          xtensa_savestate(rtcb->xcp.regs);

          /* Restore the exception context of the rtcb at the (new) head
           * of the ready-to-run task list.
           */

          rtcb = this_task();

          /* Update scheduler parameters */

          nxsched_resume_scheduler(rtcb);

          /* Then switch contexts.  Any necessary address environment
           * changes will be made when the interrupt returns.
           */

          xtensa_restorestate(rtcb->xcp.regs);
        }

      /* We are not in an interrupt handler.  Copy the user C context
       * into the TCB of the task that was previously active.  if
       * up_saveusercontext returns a non-zero value, then this is really the
       * previously running task restarting!
       */

      else if (!xtensa_context_save(rtcb->xcp.regs))
        {
#if XCHAL_CP_NUM > 0
          /* Save the co-processor state in in the suspended thread's co-
           * processor save area.
           */

          xtensa_coproc_savestate(&rtcb->xcp.cpstate);
#endif

          /* Restore the exception context of the new task that is ready to
           * run (probably tcb).  This is the new rtcb at the head of the
           * ready-to-run task list.
           */

          rtcb = this_task();

#if XCHAL_CP_NUM > 0
          /* Set up the co-processor state for the newly started thread. */

          xtensa_coproc_restorestate(&rtcb->xcp.cpstate);
#endif

#ifdef CONFIG_ARCH_ADDRENV
          /* Make sure that the address environment for the previously
           * running task is closed down gracefully (data caches dump,
           * MMU flushed) and set up the address environment for the new
           * thread at the head of the ready-to-run list.
           */

          group_addrenv(rtcb);
#endif
          /* Update scheduler parameters */

          nxsched_resume_scheduler(rtcb);

          /* Then switch contexts */

          xtensa_context_restore(rtcb->xcp.regs);
        }
    }
}
