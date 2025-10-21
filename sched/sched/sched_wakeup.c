/****************************************************************************
 * sched/sched/sched_wakeup.c
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

#include <nuttx/irq.h>
#include <nuttx/clock.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_ticksleep
 *
 * Description:
 *   The nxsched_ticksleep() function will cause the calling thread to be
 *   suspended from execution for the specified number of system ticks.
 *
 *   It can only be resumed through scheduler operations.
 *
 * Input Parameters:
 *   ticks - The number of system ticks to sleep.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_wakeup(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);

  flags = enter_critical_section();

  if (tcb->task_state == TSTATE_SLEEPING)
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Remove the task from sleeping list */

      dq_rem((FAR dq_entry_t *)tcb, list_sleepingtasks());

      wd_cancel(&tcb->waitdog);

      /* Add the task to ready-to-run task list, and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(tcb))
        {
          up_switch_context(this_task(), rtcb);
        }
    }

  leave_critical_section(flags);
}
