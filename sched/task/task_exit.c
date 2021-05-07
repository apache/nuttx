/****************************************************************************
 * sched/task/task_exit.c
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

#include  <nuttx/config.h>

#include  <sched.h>

#include  "sched/sched.h"

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
  int cpu;

  /* Get the current CPU.  By assumption, we are within a critical section
   * and, hence, the CPU index will remain stable.
   *
   * Avoid using this_task() because it may assume a state that is not
   * appropriate for an exiting task.
   */

  cpu  = this_cpu();
  dtcb = current_task(cpu);
#else
  dtcb = this_task();
#endif

  /* Update scheduler parameters */

  nxsched_suspend_scheduler(dtcb);

  /* Remove the TCB of the current task from the ready-to-run list.  A
   * context switch will definitely be necessary -- that must be done
   * by the architecture-specific logic.
   *
   * nxsched_remove_readytorun will mark the task at the head of the
   * ready-to-run with state == TSTATE_TASK_RUNNING
   */

  nxsched_remove_readytorun(dtcb);

  /* If there are any pending tasks, then add them to the ready-to-run
   * task list now
   */

  if (g_pendingtasks.head != NULL)
    {
      nxsched_merge_pending();
    }

  /* Get the new task at the head of the ready to run list */

#ifdef CONFIG_SMP
  rtcb = current_task(cpu);
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

#ifdef CONFIG_SMP
  /* Make sure that the system knows about the locked state */

  spin_setbit(&g_cpu_lockset, this_cpu(), &g_cpu_locksetlock,
              &g_cpu_schedlock);
#endif

  rtcb->task_state = TSTATE_TASK_READYTORUN;

  /* Move the TCB to the specified blocked task list and delete it.  Calling
   * nxtask_terminate with non-blocking true will suppress atexit() and
   * on-exit() calls and will cause buffered I/O to fail to be flushed.  The
   * former is required _exit() behavior; the latter is optional _exit()
   * behavior.
   */

  nxsched_add_blocked(dtcb, TSTATE_TASK_INACTIVE);

#ifdef CONFIG_SMP
  /* NOTE:
   * During nxtask_terminate(), enter_critical_section() will be called
   * to deallocate tcb. However, this would acquire g_cpu_irqlock if
   * rtcb->irqcount = 0, event though we are in critical section.
   * To prevent from acquiring, increment rtcb->irqcount here.
   */

  rtcb->irqcount++;
#endif

  ret = nxtask_terminate(dtcb->pid, true);

#ifdef CONFIG_SMP
  rtcb->irqcount--;
#endif

  rtcb->task_state = TSTATE_TASK_RUNNING;

  /* Decrement the lockcount on rctb. */

  rtcb->lockcount--;

#ifdef CONFIG_SMP
  if (rtcb->lockcount == 0)
    {
      /* Make sure that the system knows about the unlocked state */

      spin_clrbit(&g_cpu_lockset, this_cpu(), &g_cpu_locksetlock,
                  &g_cpu_schedlock);
    }
#endif

  return ret;
}
