/****************************************************************************
 * sched/task_exit.c
 *
 *   Copyright (C) 2008-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

#include  <nuttx/config.h>

#include  <sched.h>
#include  "os_internal.h"
#ifndef CONFIG_DISABLE_SIGNALS
# include "sig_internal.h"
#endif

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
 * Name: task_cancel_atexit
 *
 * Description:
 *   Cncel any registerd atexit function(s)
 *
 *   This function is called from task_exit() which implements the processor-
 *   independent part of _exit().  _exit() is, in turn, used to implement
 *   the bottom half of exit() and pthread_exit().  These cases are
 *   distinguished as follows:
 *
 *   1) _exit() should be called by user logic only from tasks.  In this
 *      case, atexit() calls will be canceled by this function.
 *   2) If the user calls exit(), the exit() function will call task_exithook()
 *      which will process all pending atexit() call.  In that case, this
 *      function will have no effect.
 *   3) If the user called pthread_exit(), the logic in this function will
 *      do nothing.  Only a task can legitimately called _exit().  atexit
 *      calls will not be cleared.  task_exithook() will be called later (from
 *      task_delete()) and if this is the final thread of the group, any
 *      registered atexit() calls will be performed.
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_ATEXIT) && !defined(CONFIG_SCHED_ONEXIT)
static inline void task_cancel_atexit(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group = tcb->group;
  DEBUGASSERT(group);

  /* This behavior applies only to tasks that call _exit() */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
    {
#if defined(CONFIG_SCHED_ATEXIT_MAX) && CONFIG_SCHED_ATEXIT_MAX > 1
      int index;

      /* Nullify each atexit function pointer */

      for (index = 0; index < CONFIG_SCHED_ATEXIT_MAX; index++)
        {
          group->tg_atexitfunc[index] = NULL;
        }
#else
      /* Nullify the atexit function to prevent its reuse. */

      group->tg_atexitfunc = NULL;
#endif
    }
}
#else
#  define task_cancel_atexit(tcb)
#endif

/****************************************************************************
 * Name: task_cancel_onexit
 *
 * Description:
 *   Cancel any registerd on)exit function(s).
 *
 *   This function is called from task_exit() which implements the processor-
 *   independent part of _exit().  _exit() is, in turn, used to implement
 *   the bottom half of exit() and pthread_exit().  These cases are
 *   distinguished as follows:
 *
 *   1) _exit() should be called by user logic only from tasks.  In this
 *      case, on_exit() calls will be canceled by this function.
 *   2) If the user calls exit(), the exit() function will call task_exithook()
 *      which will process all pending on_exit() call.  In that case, this
 *      function will have no effect.
 *   3) If the user called pthread_exit(), the logic in this function will
 *      do nothing.  Only a task can legitimately called _exit().  on_exit
 *      calls will not be cleared.  task_exithook() will be called later (from
 *      task_delete()) and if this is the final thread of the group, any
 *      registered on_exit() calls will be performed.
 *
 ****************************************************************************/
 
#ifdef CONFIG_SCHED_ONEXIT
static inline void task_cancel_onexit(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group = tcb->group;
  DEBUGASSERT(group);

  /* This behavior applies only to tasks that call _exit() */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
    {
#if defined(CONFIG_SCHED_ONEXIT_MAX) && CONFIG_SCHED_ONEXIT_MAX > 1
      int index;

      /* Nullify each atexit function pointer */

      for (index = 0; index < CONFIG_SCHED_ONEXIT_MAX; index++)
        {
          group->tg_onexitfunc[index] = NULL;
        }
#else
      group->tg_onexitfunc = NULL;
#endif
    }
}
#else
#  define task_cancel_onexit(tcb)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_exit
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
 *   Threads/tasks could also be terminated via pthread_cancel, task_delete(),
 *   and task_restart().  In the last two cases, the task will be terminated
 *   as though exit() were called.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   OK on success; or ERROR on failure
 *
 * Assumeptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

int task_exit(void)
{
  FAR struct tcb_s *dtcb = (FAR struct tcb_s*)g_readytorun.head;
  FAR struct tcb_s *rtcb;

  /* Remove the TCB of the current task from the ready-to-run list.  A context
   * switch will definitely be necessary -- that must be done by the
   * architecture-specific logic.
   *
   * sched_removereadytorun will mark the task at the head of the ready-to-run
   * with state == TSTATE_TASK_RUNNING
   */

  (void)sched_removereadytorun(dtcb);
  rtcb = (FAR struct tcb_s*)g_readytorun.head;

  /* Cancel any pending atexit() or on_exit() calls.  These are not performed
   * when performing _exit().  Different implementations of _exit() may or may
   * not* flush buffered I/O.  This implemenation *will* flush buffered I/O.
   */

   task_cancel_atexit(rtcb);
   task_cancel_onexit(rtcb);

  /* We are now in a bad state -- the head of the ready to run task list
   * does not correspond to the thread that is running.  Disabling pre-
   * emption on this TCB and marking the new ready-to-run task as not
   * running (see, for example, get_errno_ptr()).
   *
   * We disable pre-emption here by directly incrementing the lockcount
   * (vs. calling sched_lock()).
   */

  rtcb->lockcount++;
  rtcb->task_state = TSTATE_TASK_READYTORUN;

  /* Move the TCB to the specified blocked task list and delete it */

  sched_addblocked(dtcb, TSTATE_TASK_INACTIVE);
  task_delete(dtcb->pid);
  rtcb->task_state = TSTATE_TASK_RUNNING;

  /* If there are any pending tasks, then add them to the ready-to-run
   * task list now
   */

  if (g_pendingtasks.head)
    {
      (void)sched_mergepending();
    }

  /* We can't use sched_unlock() to decrement the lock count because the
   * sched_mergepending() call above might have changed the task at the
   * head of the ready-to-run list.  Furthermore, we should not need to
   * perform the unlock action anyway because we know that the pending
   * task list is empty.  So all we really need to do is to decrement
   * the lockcount on rctb.
   */

  rtcb->lockcount--;
  return OK;
}
