/****************************************************************************
 * sched/task_restart.c
 *
 *   Copyright (C) 2007, 2009, 2012-2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>

#include "os_internal.h"
#include "group_internal.h"
#include "sig_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
 * Inputs:
 *   pid - The task ID of the task to delete.  An ID of zero signifies the
 *         calling task.
 *
 * Return Value:
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
  irqstate_t state;
  int status;

  /* Make sure this task does not become ready-to-run while
   * we are futzing with its TCB
   */

  sched_lock();

  /* Check if the task to restart is the calling task */

  rtcb = (FAR struct tcb_s *)g_readytorun.head;
  if ((pid == 0) || (pid == rtcb->pid))
    {
      /* Not implemented */

      set_errno(ENOSYS);
      return ERROR;
    }

  /* We are restarting some other task than ourselves */

  else
    {
      /* Find for the TCB associated with matching pid  */

      tcb = (FAR struct task_tcb_s *)sched_gettcb(pid);
#ifndef CONFIG_DISABLE_PTHREAD
      if (!tcb || (tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
#else
      if (!tcb)
#endif
        {
          /* There is no TCB with this pid or, if there is, it is not a
           * task.
           */

          set_errno(ESRCH);
          return ERROR;
        }

      /* Try to recover from any bad states */
 
      task_recover((FAR struct tcb_s *)tcb);

      /* Kill any children of this thread */

#if HAVE_GROUP_MEMBERS
      (void)group_killchildren(tcb);
#endif

      /* Remove the TCB from whatever list it is in.  At this point, the
       * TCB should no longer be accessible to the system 
       */

      state = irqsave();
      dq_rem((FAR dq_entry_t*)tcb,
             (dq_queue_t*)g_tasklisttable[tcb->cmn.task_state].list);
      tcb->cmn.task_state = TSTATE_TASK_INVALID;
      irqrestore(state);

      /* Deallocate anything left in the TCB's queues */

      sig_cleanup((FAR struct tcb_s *)tcb); /* Deallocate Signal lists */

      /* Reset the current task priority  */

      tcb->cmn.sched_priority = tcb->init_priority;

      /* Reset the base task priority and the number of pending reprioritizations */

#ifdef CONFIG_PRIORITY_INHERITANCE
      tcb->cmn.base_priority = tcb->init_priority;
#  if CONFIG_SEM_NNESTPRIO > 0
      tcb->cmn.npend_reprio = 0;
#  endif
#endif

      /* Re-initialize the processor-specific portion of the TCB
       * This will reset the entry point and the start-up parameters
       */

      up_initial_state((FAR struct tcb_s *)tcb);

      /* Add the task to the inactive task list */

      dq_addfirst((FAR dq_entry_t*)tcb, (dq_queue_t*)&g_inactivetasks);
      tcb->cmn.task_state = TSTATE_TASK_INACTIVE;

      /* Activate the task */

      status = task_activate((FAR struct tcb_s *)tcb);
      if (status != OK)
        {
          (void)task_delete(pid);
          set_errno(-status);
          return ERROR;
        }
    }

  sched_unlock();
  return OK;
}
