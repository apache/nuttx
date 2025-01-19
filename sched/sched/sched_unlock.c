/****************************************************************************
 * sched/sched/sched_unlock.c
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

#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_unlock
 *
 * Description:
 *   This function decrements the preemption lock count.  Typically this
 *   is paired with sched_lock() and concludes a critical section of
 *   code.  Preemption will not be unlocked until sched_unlock() has
 *   been called as many times as sched_lock().  When the lockcount is
 *   decremented to zero, any tasks that were eligible to preempt the
 *   current task will execute.
 *
 ****************************************************************************/

void sched_unlock(void)
{
  /* sched_unlock should have no effect if called from the interrupt level. */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

      /* rtcb may be NULL only during early boot-up phases */

      DEBUGASSERT(rtcb == NULL || rtcb->lockcount > 0);

      /* Check if the lock counter has decremented to zero. If so,
       * then pre-emption has been re-enabled.
       */

      if (rtcb != NULL && --rtcb->lockcount == 0)
        {
          irqstate_t flags = enter_critical_section_wo_note();

          /* Note that we no longer have pre-emption disabled. */

          nxsched_critmon_preemption(rtcb, false, return_address(0));
          sched_note_preemption(rtcb, false);

          /* Release any ready-to-run tasks that have collected in
           * g_pendingtasks.
           *
           * NOTE: This operation has a very high likelihood of causing
           * this task to be switched out!
           */

          if (list_pendingtasks()->head != NULL)
            {
              if (nxsched_merge_pending())
                {
                  up_switch_context(this_task(), rtcb);
                }
            }

#if CONFIG_RR_INTERVAL > 0
          /* If (1) the task that was running supported round-robin
           * scheduling and (2) if its time slice has already expired, but
           * (3) it could not slice out because pre-emption was disabled,
           * then we need to swap the task out now and reassess the interval
           * timer for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR &&
              rtcb->timeslice == 0)
            {
              /* Yes.. that is the situation.  But one more thing.  The call
               * to nxsched_merge_pending() above may have actually replaced
               * the task at the head of the ready-to-run list.  In that
               * case, we need only to reset the timeslice value back to the
               * maximum.
               */

              if (rtcb != this_task())
                {
                  rtcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
                }
#  ifdef CONFIG_SCHED_TICKLESS
              else if ((rtcb->flags & TCB_FLAG_PREEMPT_SCHED) == 0)
                {
                  rtcb->flags |= TCB_FLAG_PREEMPT_SCHED;
                  nxsched_reassess_timer();
                  rtcb->flags &= ~TCB_FLAG_PREEMPT_SCHED;
                }
#  endif
            }
#endif

#ifdef CONFIG_SCHED_SPORADIC
#  if CONFIG_RR_INTERVAL > 0
          else
#  endif
          /* If (1) the task that was running supported sporadic scheduling
           * and (2) if its budget slice has already expired, but (3) it
           * could not slice out because pre-emption was disabled, then we
           * need to swap the task out now and reassess the interval timer
           * for the next time slice.
           */

          if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC
              && rtcb->timeslice < 0)
            {
              /* Yes.. that is the situation.  Force the low-priority state
               * now
               */

              nxsched_sporadic_lowpriority(rtcb);

#  ifdef CONFIG_SCHED_TICKLESS
              /* Make sure that the call to nxsched_merge_pending() did not
               * change the currently active task.
               */

              if (rtcb == this_task() &&
                  (rtcb->flags & TCB_FLAG_PREEMPT_SCHED) == 0)
                {
                  rtcb->flags |= TCB_FLAG_PREEMPT_SCHED;
                  nxsched_reassess_timer();
                  rtcb->flags &= ~TCB_FLAG_PREEMPT_SCHED;
                }
#  endif
            }
#endif

          leave_critical_section_wo_note(flags);
        }
    }
}
