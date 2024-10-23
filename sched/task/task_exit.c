/****************************************************************************
 * sched/task/task_exit.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/sched_note.h>

#include "sched/sched.h"

#ifdef CONFIG_SMP
#  include "irq/irq.h"
#endif

#include "signal/signal.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_exit
 *
 * Description:
 *   This is a part of the logic used to implement _exit().  The full
 *   implementation of _exit() is architecture-dependent. The _exit()
 *   function also implements the bottom half of exit() and pthread_exit().
 *
 *   This function causes the currently running task (i.e., the task at the
 *   head of the ready-to-run list) to cease to exist.  This function should
 *   never be called from normal user code, but only from the architecture-
 *   specific implementation of exit.
 *
 *   Threads/tasks could also be terminated via pthread_cancel,
 *   task_delete(), and task_restart().  In the last two cases, the
 *   task will be terminated as though exit() were called.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; or ERROR on failure
 *
 * Assumptions:
 *   Executing within a critical section established by the caller.
 *
 ****************************************************************************/

int nxtask_exit(void)
{
  FAR struct tcb_s *dtcb;
  FAR struct tcb_s *rtcb;
  int ret;
#ifdef CONFIG_SMP
  /* Avoid using this_task() because it may assume a state that is not
   * appropriate for an exiting task.
   */

  dtcb = current_task(this_cpu());
#else
  dtcb = this_task();
#endif

  sinfo("%s pid=%d,TCB=%p\n", get_task_name(dtcb),
        dtcb->pid, dtcb);

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(dtcb);

  /* Remove the TCB of the current task from the ready-to-run list.  A
   * context switch will definitely be necessary -- that must be done
   * by the architecture-specific logic.
   *
   * nxsched_remove_readytorun will mark the task at the head of the
   * ready-to-run with state == TSTATE_TASK_RUNNING
   */

  nxsched_remove_self(dtcb);

  /* Get the new task at the head of the ready to run list */

#ifdef CONFIG_SMP
  rtcb = current_task(this_cpu());
#else
  rtcb = this_task();
#endif

  /* NOTE: nxsched_resume_scheduler() was moved to up_exit()
   * because the global IRQ control for SMP should be deferred until
   * context switching, otherwise, the context switching would be done
   * without a critical section
   */

  /* We are now in a bad state -- the head of the ready to run task list
   * does not correspond to the thread that is running.  Disabling pre-
   * emption on this TCB and marking the new ready-to-run task as not
   * running.
   *
   * We disable pre-emption here by directly incrementing the lockcount
   * (vs. calling sched_lock()).
   */

  rtcb->lockcount++;

  rtcb->task_state = TSTATE_TASK_READYTORUN;

#ifdef CONFIG_SMP
  /* NOTE:
   * During nxtask_terminate(), enter_critical_section() will be called
   * to deallocate tcb. However, this would acquire g_cpu_irqlock if
   * rtcb->irqcount = 0, event though we are in critical section.
   * To prevent from acquiring, increment rtcb->irqcount here.
   */

  rtcb->irqcount++;
#endif

  dtcb->task_state = TSTATE_TASK_INACTIVE;
  sched_note_stop(dtcb);
  ret = nxsched_release_tcb(dtcb, dtcb->flags & TCB_FLAG_TTYPE_MASK);

#ifdef CONFIG_SMP
  rtcb->irqcount--;
#endif

  rtcb->task_state = TSTATE_TASK_RUNNING;

  /* Decrement the lockcount on rctb. */

  rtcb->lockcount--;

  return ret;
}
