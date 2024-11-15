/****************************************************************************
 * sched/sched/sched_suspend.c
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

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP
/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct suspend_arg_s
{
  pid_t pid;
  cpu_set_t saved_affinity;
  bool need_restore;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nxsched_suspend_handler(FAR void *cookie)
{
  FAR struct suspend_arg_s *arg = cookie;
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();
  tcb = nxsched_get_tcb(arg->pid);

  if (!tcb || tcb->task_state == TSTATE_TASK_INVALID ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      leave_critical_section(flags);
      return OK;
    }

  if (arg->need_restore)
    {
      tcb->affinity = arg->saved_affinity;
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  nxsched_remove_readytorun(tcb);

  tcb->task_state = TSTATE_TASK_STOPPED;
  dq_addlast((FAR dq_entry_t *)tcb, &g_stoppedtasks);

  leave_critical_section(flags);
  return OK;
}
#endif

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
  bool switch_needed;

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

      tcb->task_state = TSTATE_TASK_STOPPED;
      dq_addlast((FAR dq_entry_t *)tcb, list_stoppedtasks());
    }
  else
    {
      FAR struct tcb_s *rtcb = this_task();

      /* The task was running or runnable before being stopped.  Simply
       * block it in the stopped state.  If tcb refers to this task, then
       * this action will block this task.
       * Before doing that make sure this is not the idle task,
       * descheduling that isn't going to end well.
       */

      DEBUGASSERT(!is_idle_task(tcb));

      /* Remove the tcb task from the ready-to-run list. */

#ifdef CONFIG_SMP
      if (tcb->task_state == TSTATE_TASK_RUNNING && tcb->cpu != this_cpu())
        {
          struct suspend_arg_s arg;

          if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
            {
              arg.pid = tcb->pid;
              arg.need_restore = false;
            }
          else
            {
              arg.pid = tcb->pid;
              arg.saved_affinity = tcb->affinity;
              arg.need_restore = true;

              tcb->flags |= TCB_FLAG_CPU_LOCKED;
              CPU_SET(tcb->cpu, &tcb->affinity);
            }

          nxsched_smp_call_single(tcb->cpu, nxsched_suspend_handler, &arg);
        }
      else
#endif
        {
          switch_needed = nxsched_remove_readytorun(tcb);

          if (list_pendingtasks()->head)
            {
              switch_needed |= nxsched_merge_pending();
            }

          /* Add the task to the specified blocked task list */

          tcb->task_state = TSTATE_TASK_STOPPED;
          dq_addlast((FAR dq_entry_t *)tcb, list_stoppedtasks());

          /* Now, perform the context switch if one is needed */

          if (switch_needed)
            {
              up_switch_context(this_task(), rtcb);
            }
        }
    }

  leave_critical_section(flags);
}
