/****************************************************************************
 * arch/z16/src/common/z16_blocktask.c
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

#include <stdbool.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of the ready to run list must
 *   be stopped.  Save its context and move it to the inactive list
 *   specified by task_state.
 *
 * Input Parameters:
 *   tcb: Refers to a task in the ready-to-run list (normally the task at
 *     the head of the list).  It must be stopped, its context saved and
 *     moved into one of the waiting task lists.  If it was the task at the
 *     head of the ready-to-run list, then a context switch to the new
 *     ready to run task must be performed.
 *   task_state: Specifies which waiting task list should hold the blocked
 *     task TCB.
 *
 ****************************************************************************/

void up_block_task(FAR struct tcb_s *tcb, tstate_t task_state)
{
  FAR struct tcb_s *rtcb = this_task();
  bool switch_needed;

  /* Verify that the context switch can be performed */

  DEBUGASSERT((tcb->task_state >= FIRST_READY_TO_RUN_STATE) &&
              (tcb->task_state <= LAST_READY_TO_RUN_STATE));

  /* sinfo("Blocking TCB=%p\n", tcb); */

  /* Remove the tcb task from the ready-to-run list.  If we are blocking the
   * task at the head of the task list (the most likely case), then a
   * context switch to the next ready-to-run task is needed. In this case,
   * it should also be true that rtcb == tcb.
   */

  switch_needed = nxsched_remove_readytorun(tcb);

  /* Add the task to the specified blocked task list */

  nxsched_add_blocked(tcb, (tstate_t)task_state);

  /* If there are any pending tasks, then add them to the ready-to-run
   * task list now
   */

  if (g_pendingtasks.head)
    {
      switch_needed |= nxsched_merge_pending();
    }

  /* Now, perform the context switch if one is needed */

  if (switch_needed)
    {
      /* Update scheduler parameters */

      nxsched_suspend_scheduler(rtcb);

      /* Are we in an interrupt handler? */

      if (IN_INTERRUPT)
        {
          /* Yes, then we have to do things differently.
           * Just copy the current registers into the OLD rtcb.
           */

          SAVE_IRQCONTEXT(rtcb);

          /* Restore the exception context of the rtcb at the (new) head
           * of the ready-to-run task list.
           */

          rtcb = this_task();

          /* Reset scheduler parameters */

          nxsched_resume_scheduler(rtcb);

          /* Then setup so that the context will be performed on exit
           * from the interrupt.
           */

          SET_IRQCONTEXT(rtcb);
        }

      /* Copy the user C context into the TCB at the (old) head of the
       * ready-to-run Task list. if SAVE_USERCONTEXT returns a non-zero
       * value, then this is really the previously running task restarting!
       */

      else if (!SAVE_USERCONTEXT(rtcb))
        {
          /* Restore the exception context of the rtcb at the (new) head
           * of the ready-to-run task list.
           */

          rtcb = this_task();

          /* Reset scheduler parameters */

          nxsched_resume_scheduler(rtcb);

          /* Then switch contexts */

          RESTORE_USERCONTEXT(rtcb);
        }
    }
}
