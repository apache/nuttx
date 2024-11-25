/****************************************************************************
 * sched/sched/sched_lock.c
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_lock
 *
 * Description:
 *   This function disables context switching by disabling addition of
 *   new tasks to the g_readytorun task list.  The task that calls this
 *   function will be the only task that is allowed to run until it
 *   either calls  sched_unlock() (the appropriate number of times) or
 *   until it blocks itself.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SMP

int sched_lock(void)
{
  FAR struct tcb_s *rtcb;

  /* If the CPU supports suppression of interprocessor interrupts, then
   * simple disabling interrupts will provide sufficient protection for
   * the following operation.
   */

  rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      irqstate_t flags;

      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);

      flags = enter_critical_section();

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread
       */

      rtcb->lockcount++;

      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
          nxsched_critmon_preemption(rtcb, true, return_address(0));
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_preemption(rtcb, true);
#endif
        }

      /* Move any tasks in the ready-to-run list to the pending task list
       * where they will not be available to run until the scheduler is
       * unlocked and nxsched_merge_pending() is called.
       */

      nxsched_merge_prioritized(list_readytorun(),
                                list_pendingtasks(),
                                TSTATE_TASK_PENDING);

      leave_critical_section(flags);
    }

  return OK;
}

#else /* CONFIG_SMP */

int sched_lock(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread (on any CPU)
       */

      rtcb->lockcount++;

      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
          nxsched_critmon_preemption(rtcb, true, return_address(0));
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_preemption(rtcb, true);
#endif
        }
    }

  return OK;
}

#endif /* CONFIG_SMP */
