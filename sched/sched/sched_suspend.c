/****************************************************************************
 * sched/sched/sched_suspend.c
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

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_suspend
 *
 * Description:
 *   Suspend/pause the specified thread.  This is normally calling indirectly
 *   via group_suspend_children();
 *
 ****************************************************************************/

void nxsched_suspend(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);

  flags = enter_critical_section();

  /* Check the current state of the task */

  if (tcb->task_state >= FIRST_BLOCKED_STATE &&
      tcb->task_state <= LAST_BLOCKED_STATE)
    {
      /* Remove the TCB from the the blocked task list. */

      nxsched_remove_blocked(tcb);

      /* Set the errno value to EINTR.  The task will be restarted in the
       * running or runnable state and will appear to have awakened from
       * the block state by a signal.
       */

      tcb->errcode = EINTR;

      /* Move the TCB to the g_stoppedtasks list. */

      nxsched_add_blocked(tcb, TSTATE_TASK_STOPPED);
    }
  else
    {
      /* The task was running or runnable before being stopped.  Simply
       * block it in the stopped state.  If tcb refers to this task, then
       * this action will block this task.
       * Before doing that make sure this is not the idle task,
       * descheduling that isn't going to end well.
       */

      DEBUGASSERT(NULL != tcb->flink);
      up_block_task(tcb, TSTATE_TASK_STOPPED);
    }

  leave_critical_section(flags);
}
