/****************************************************************************
 * sched/task/task_restart.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"
#include "task/task.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

#ifdef CONFIG_SMP
struct restart_arg_s
{
  pid_t pid;
  cpu_set_t saved_affinity;
  bool need_restore;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int restart_handler(FAR void *cookie)
{
  FAR struct restart_arg_s *arg = cookie;
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();

  /* tcb that we want restart */

  tcb = nxsched_get_tcb(arg->pid);
  if (!tcb || tcb->task_state == TSTATE_TASK_INVALID ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      leave_critical_section(flags);
      return -ESRCH;
    }

  if (arg->need_restore)
    {
      tcb->affinity = arg->saved_affinity;
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  nxsched_remove_readytorun(tcb);

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: nxtask_reset_task
 *
 * Description:
 *   We use this function to reset tcb
 *
 ****************************************************************************/

static void nxtask_reset_task(FAR struct tcb_s *tcb, bool remove)
{
  /* Try to recover from any bad states */

  nxtask_recover(tcb);

  /* Kill any children of this thread */

#ifdef HAVE_GROUP_MEMBERS
  group_kill_children(tcb);
#endif

  /* Remove the TCB from whatever list it is in.  After this point, the TCB
   * should no longer be accessible to the system
   */

  if (remove)
    {
      nxsched_remove_readytorun(tcb);
    }

  /* Deallocate anything left in the TCB's signal queues */

  nxsig_cleanup(tcb);             /* Deallocate Signal lists */
  sigemptyset(&tcb->sigprocmask); /* Reset sigprocmask */

  /* Reset the current task priority  */

  tcb->sched_priority = tcb->init_priority;

  /* The task should restart with pre-emption disabled and not in a critical
   * section.
   */

  tcb->lockcount = 0;
#ifdef CONFIG_SMP
  tcb->irqcount  = 0;
#endif

  /* Reset the base task priority and the number of pending
   * reprioritizations.
   */

#ifdef CONFIG_PRIORITY_INHERITANCE
  tcb->base_priority = tcb->init_priority;
  tcb->boost_priority = 0;
#endif

  /* Re-initialize the processor-specific portion of the TCB.  This will
   * reset the entry point and the start-up parameters
   */

  up_initial_state(tcb);

  /* Add the task to the inactive task list */

  dq_addfirst((FAR dq_entry_t *)tcb, list_inactivetasks());
  tcb->task_state = TSTATE_TASK_INACTIVE;
}

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

static int nxtask_restart(pid_t pid)
{
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *tcb;
  irqstate_t flags;
  int ret;

  /* We are restarting some other task than ourselves.  Make sure that the
   * task does not change its state while we are executing.  In the single
   * CPU state this could be done by disabling pre-emption.  But we will
   * a little stronger medicine on the SMP case:  The task make be running
   * on another CPU.
   */

  flags = enter_critical_section();

  /* Check if the task to restart is the calling task */

  rtcb = this_task();
  if (pid == 0 || pid == rtcb->pid)
    {
      /* Not implemented */

      ret = -ENOSYS;
      goto errout_with_lock;
    }

  /* Find for the TCB associated with matching pid  */

  tcb = nxsched_get_tcb(pid);
#ifndef CONFIG_DISABLE_PTHREAD
  if (!tcb || (tcb->flags & TCB_FLAG_TTYPE_MASK) ==
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
  if (tcb->task_state == TSTATE_TASK_RUNNING &&
      tcb->cpu != this_cpu())
    {
      struct restart_arg_s arg;

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

      nxsched_smp_call_single(tcb->cpu, restart_handler, &arg);

      tcb = nxsched_get_tcb(pid);
      if (!tcb || tcb->task_state != TSTATE_TASK_INVALID ||
          (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
        {
          ret = -ESRCH;
          goto errout_with_lock;
        }

      DEBUGASSERT(tcb->task_state != TSTATE_TASK_RUNNING);
      nxtask_reset_task(tcb, false);
      leave_critical_section(flags);

      /* Activate the task. */

      nxtask_activate(tcb);

      return OK;
    }
#endif /* CONFIG_SMP */

  nxtask_reset_task(tcb, true);
  leave_critical_section(flags);

  /* Activate the task. */

  nxtask_activate(tcb);
  return OK;

errout_with_lock:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
