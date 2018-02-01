/****************************************************************************
 * sched/task/task_restart.c
 *
 *   Copyright (C) 2007, 2009, 2012-2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 *   OK on sucess; ERROR on failure.
 *
 *   This function can fail if:
 *   (1) A pid of zero or the pid of the calling task is provided
 *      (functionality not implemented)
 *   (2) The pid is not associated with any task known to the system.
 *
 ****************************************************************************/

int task_restart(pid_t pid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_tcb_s *tcb;
  FAR dq_queue_t *tasklist;
  irqstate_t flags;
  int errcode;
#ifdef CONFIG_SMP
  int cpu;
#endif
  int ret;

  /* Check if the task to restart is the calling task */

  rtcb = this_task();
  if ((pid == 0) || (pid == rtcb->pid))
    {
      /* Not implemented */

      errcode = ENOSYS;
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

  tcb = (FAR struct task_tcb_s *)sched_gettcb(pid);
#ifndef CONFIG_DISABLE_PTHREAD
  if (!tcb || (tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
#else
  if (!tcb)
#endif
    {
      /* There is no TCB with this pid or, if there is, it is not a task. */

      errcode = ESRCH;
      goto errout_with_lock;
    }

#ifdef CONFIG_SMP
  /* If the task is running on another CPU, then pause that CPU.  We can
   * then manipulate the TCB of the restarted task and when we resume the
   * that CPU, the restart take effect.
   */

  cpu = sched_cpu_pause(&tcb->cmn);
#endif /* CONFIG_SMP */

  /* Try to recover from any bad states */

  task_recover((FAR struct tcb_s *)tcb);

  /* Kill any children of this thread */

#ifdef HAVE_GROUP_MEMBERS
  (void)group_killchildren(tcb);
#endif

  /* Remove the TCB from whatever list it is in.  After this point, the TCB
   * should no longer be accessible to the system
   */

#ifdef CONFIG_SMP
  tasklist = TLIST_HEAD(tcb->cmn.task_state, tcb->cmn.cpu);
#else
  tasklist = TLIST_HEAD(tcb->cmn.task_state);
#endif

  dq_rem((FAR dq_entry_t *)tcb, tasklist);
  tcb->cmn.task_state = TSTATE_TASK_INVALID;

  /* Deallocate anything left in the TCB's queues */

  nxsig_cleanup((FAR struct tcb_s *)tcb); /* Deallocate Signal lists */

  /* Reset the current task priority  */

  tcb->cmn.sched_priority = tcb->cmn.init_priority;

  /* The task should restart with pre-emption disabled and not in a critical
   * secton.
   */

  tcb->cmn.lockcount = 0;
#ifdef CONFIG_SMP
  tcb->cmn.irqcount  = 0;
#endif

  /* Reset the base task priority and the number of pending reprioritizations */

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

  dq_addfirst((FAR dq_entry_t *)tcb, (FAR dq_queue_t *)&g_inactivetasks);
  tcb->cmn.task_state = TSTATE_TASK_INACTIVE;

#ifdef CONFIG_SMP
  /* Resume the paused CPU (if any) */

  if (cpu >= 0)
    {
      ret = up_cpu_resume(cpu);
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_lock;
        }
    }
#endif /* CONFIG_SMP */

  leave_critical_section(flags);

  /* Activate the task. */

  ret = task_activate((FAR struct tcb_s *)tcb);
  if (ret != OK)
    {
      (void)task_terminate(pid, true);
      errcode = -ret;
      goto errout_with_lock;
    }

  return OK;

errout_with_lock:
  leave_critical_section(flags);
errout:
  set_errno(errcode);
  return ERROR;
}
