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

void sched_lock(void)
{
  /* sched_lock() should have no effect if called from the interrupt level. */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

      DEBUGASSERT(rtcb == NULL || rtcb->lockcount < MAX_LOCK_COUNT);

      /* A counter is used to support locking. This allows nested lock
       * operations on this thread (on any CPU)
       */

      if (rtcb != NULL && rtcb->lockcount++ == 0)
        {
#if (CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0) || \
    defined(CONFIG_SCHED_INSTRUMENTATION_PREEMPTION)
          irqstate_t flags = enter_critical_section_wo_note();

          /* Note that we have pre-emption locked */

          nxsched_critmon_preemption(rtcb, true, return_address(0));
          sched_note_preemption(rtcb, true);
          leave_critical_section_wo_note(flags);
#endif
        }
    }
}
