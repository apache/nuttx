/****************************************************************************
 * sched/task/task_restart.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_restart
 *
 * Description:
 *   This function "restarts" a task.  The task is first terminated and then
 *   reinitialized with same ID, priority, original entry point, stack size,
 *   and parameters it had when it was first started.
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  An ID of zero signifies the
 *         calling task.
 *
 * Returned Value:
 *   Zero (OK) on success; or negated errno on failure
 *
 *   This function can fail if:
 *   (1) A pid of zero or the pid of the calling task is provided
 *      (functionality not implemented)
 *   (2) The pid is not associated with any task known to the system.
 *
 ****************************************************************************/

int nxtask_restart(pid_t pid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_tcb_s *tcb;
  FAR dq_queue_t *tasklist;
  irqstate_t flags;
  int ret;
#ifdef CONFIG_SMP
  int cpu;
#endif

  /* Check if the task to restart is the calling task */

  rtcb = this_task();
  if ((pid == 0) || (pid == rtcb->pid))
    {
      /* Not implemented */

      ret = -ENOSYS;
      goto errout;
    }

  /* We are restarting some other task than ourselves.  Make sure that the
   * task does not change its state while we are executing.  In the single
   * CPU state this could be done by disabling pre-emption.  But we will
   * a little stronger medicine on the SMP case:  The task make be running
   * on another CPU.
   */

  flags = enter_critical_section();

  /* Find for the TCB associated with matching pid  */

  tcb = (FAR struct task_tcb_s *)nxsched_get_tcb(pid);
#ifndef CONFIG_DISABLE_PTHREAD
  if (!tcb || (tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) ==
      TCB_FLAG_TTYPE_PTHREAD)
#else
  if (!tcb)
#endif
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      ret = -ESRCH;
      goto errout_with_lock;
    }

#ifdef CONFIG_SMP
  /* If the task is running on another CPU, then pause that CPU.  We can
   * then manipulate the TCB of the restarted task and when we resume the
   * that CPU, the restart take effect.
   */

  cpu = nxsched_pause_cpu(&tcb->cmn);
#endif /* CONFIG_SMP */

  /* Try to recover from any bad states */

  nxtask_recover((FAR struct tcb_s *)tcb);

  /* Kill any children of this thread */

#ifdef HAVE_GROUP_MEMBERS
  group_kill_children((FAR struct tcb_s *)tcb);
#endif

  /* Remove the TCB from whatever list it is in.  After this point, the TCB
   * should no longer be accessible to the system
   */

#ifdef CONFIG_SMP
  tasklist = TLIST_HEAD(&tcb->cmn, tcb->cmn.cpu);
#else
  tasklist = TLIST_HEAD(&tcb->cmn);
#endif

  dq_rem((FAR dq_entry_t *)tcb, tasklist);
  tcb->cmn.task_state = TSTATE_TASK_INVALID;

  /* Deallocate anything left in the TCB's signal queues */

  nxsig_cleanup((FAR struct tcb_s *)tcb);  /* Deallocate Signal lists */
  tcb->cmn.sigprocmask = NULL_SIGNAL_SET;  /* Reset sigprocmask */

  /* Reset the current task priority  */

  tcb->cmn.sched_priority = tcb->cmn.init_priority;

  /* The task should restart with pre-emption disabled and not in a critical
   * section.
   */

  tcb->cmn.lockcount = 0;
#ifdef CONFIG_SMP
  tcb->cmn.irqcount  = 0;
#endif

  /* Reset the base task priority and the number of pending
   * reprioritizations.
   */

#ifdef CONFIG_PRIORITY_INHERITANCE
  tcb->cmn.base_priority = tcb->cmn.init_priority;
#  if CONFIG_SEM_NNESTPRIO > 0
  tcb->cmn.npend_reprio = 0;
#  endif
#endif

  /* Re-initialize the processor-specific portion of the TCB.  This will
   * reset the entry point and the start-up parameters
   */

  up_initial_state((FAR struct tcb_s *)tcb);

  /* Add the task to the inactive task list */

  dq_addfirst((FAR dq_entry_t *)tcb, &g_inactivetasks);
  tcb->cmn.task_state = TSTATE_TASK_INACTIVE;

#ifdef CONFIG_SMP
  /* Resume the paused CPU (if any) */

  if (cpu >= 0)
    {
      ret = up_cpu_resume(cpu);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }
#endif /* CONFIG_SMP */

  leave_critical_section(flags);

  /* Activate the task. */

  nxtask_activate((FAR struct tcb_s *)tcb);
  return OK;

errout_with_lock:
  leave_critical_section(flags);
errout:
  return ret;
}

/****************************************************************************
 * Name: task_restart
 *
 * Description:
 *   This function "restarts" a task.  The task is first terminated and then
 *   reinitialized with same ID, priority, original entry point, stack size,
 *   and parameters it had when it was first started.
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  An ID of zero signifies the
 *         calling task.
 *
 * Returned Value:
 *   Zero (OK) on success; -1 (ERROR) on failure with the errno variable set
 *   appropriately.
 *
 *   This function can fail if:
 *   (1) A pid of zero or the pid of the calling task is provided
 *      (functionality not implemented)
 *   (2) The pid is not associated with any task known to the system.
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
int task_restart(pid_t pid)
{
  int ret = nxtask_restart(pid);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
#endif
