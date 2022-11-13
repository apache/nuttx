/****************************************************************************
 * arch/misoc/src/minerva/minerva_reprioritizertr.c
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
#include <stdbool.h>
#include <sched.h>
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "minerva.h"

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
 *      priority of the current, running task and it now has the priority.
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
       * nxsched_remove_readytorun() will return true if we just remove the
       * head of the ready to run list.
       */

      switch_needed = nxsched_remove_readytorun(tcb, false);

      /* Setup up the new task priority */

      tcb->sched_priority = (uint8_t) priority;

      /* Return the task to the specified blocked task list.
       * nxsched_add_readytorun will return true if the task was added to the
       * new list.  We will need to perform a context switch only if the
       * EXCLUSIVE or of the two calls is non-zero (i.e., one and only one
       * the calls changes the head of the ready-to-run list).
       */

      switch_needed ^= nxsched_add_readytorun(tcb);

      /* Now, perform the context switch if one is needed */

      if (switch_needed)
        {
          /* If we are going to do a context switch, then now is the right
           * time to add any pending tasks back into the ready-to-run list.
           */

          if (g_pendingtasks.head)
            {
              nxsched_merge_pending();
            }

          /* Update scheduler parameters */

          nxsched_suspend_scheduler(rtcb);

          /* Are we in an interrupt handler? */

          if (g_current_regs)
            {
              /* Yes, then we have to do things differently. Just copy the
               * g_current_regs into the OLD rtcb.
               */

              up_savestate(rtcb->xcp.regs);

              /* Restore the exception context of the rtcb at the (new) head
               * of the ready-to-run task list.
               */

              rtcb = this_task();

              /* Update scheduler parameters */

              nxsched_resume_scheduler(rtcb);

              /* Then switch contexts.  Any necessary address environment
               * changes will be made when the interrupt returns.
               */

              up_restorestate(rtcb->xcp.regs);
            }

          /* No, then we will need to perform the user context switch */

          else
            {
              /* Switch context to the context of the task at the head of the
               * ready to run list.
               */

              struct tcb_s *nexttcb = this_task();

              /* Update scheduler parameters */

              nxsched_resume_scheduler(nexttcb);

              /* Then switch contexts */

              up_switchcontext(rtcb->xcp.regs, nexttcb->xcp.regs);

              /* up_switchcontext forces a context switch to the task at the
               * head of the ready-to-run list.  It does not 'return' in the
               * normal sense.  When it does return, it is because the
               * blocked task is again ready to run and has execution
               * priority.
               */
            }
        }
    }
}
