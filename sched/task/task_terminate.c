/*******************************************************************************
 * sched/task/task_terminate.c
 *
 *   Copyright (C) 2007-2009, 2011-2014, 2016 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "signal/signal.h"
#include "task/task.h"

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: nxtask_terminate
 *
 * Description:
 *   This function causes a specified task to cease to exist.  Its stack and
 *   TCB will be deallocated.  This function is the internal implementation
 *   of the task_delete() function.  It includes and additional parameter
 *   to determine if blocking is permitted or not.
 *
 *   This function is the final function called all task termination
 *   sequences.  nxtask_terminate() is called only from task_delete() (with
 *   nonblocking == false) and from nxtask_exit() (with nonblocking == true).
 *
 *   The path through nxtask_exit() supports the final stops of the exit(),
 *   _exit(), and pthread_exit
 *
 *   - pthread_exit().  Calls _exit()
 *   - exit(). Calls _exit()
 *   - _exit().  Calls nxtask_exit() making the currently running task
 *     non-running. nxtask_exit then calls nxtask_terminate() (with nonblocking
 *     == true) to terminate the non-running task.
 *
 *   NOTE: that the state of non-blocking is irrelevant when called through
 *   exit() and pthread_exit().  In those cases nxtask_exithook() has already
 *   been called with nonblocking == false;
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *   nonblocking - True: The task is an unhealthy, partially torn down
 *         state and is not permitted to block.
 *
 * Returned Value:
 *   OK on success; or ERROR on failure
 *
 *   This function can fail if the provided pid does not correspond to a
 *   task (errno is not set)
 *
 *******************************************************************************/

int nxtask_terminate(pid_t pid, bool nonblocking)
{
  FAR struct tcb_s *dtcb;
  FAR dq_queue_t *tasklist;
  irqstate_t flags;
#ifdef CONFIG_SMP
  int cpu;
#endif
  int ret;

  /* Make sure the task does not become ready-to-run while we are futzing
   * with its TCB.  Within the critical section, no new task may be started
   * or terminated (even in the SMP case).
   */

  flags = enter_critical_section();

  /* Find for the TCB associated with matching PID */

  dtcb = nxsched_get_tcb(pid);
  if (!dtcb)
    {
      /* This PID does not correspond to any known task */

      ret = -ESRCH;
      goto errout_with_lock;
    }

  /* Verify our internal sanity */

#ifdef CONFIG_SMP
  DEBUGASSERT(dtcb->task_state < NUM_TASK_STATES);
#else
  DEBUGASSERT(dtcb->task_state != TSTATE_TASK_RUNNING &&
              dtcb->task_state < NUM_TASK_STATES);
#endif

  /* Remove the task from the OS's task lists.  We must be in a critical
   * section and the must must not be running to do this.
   */

#ifdef CONFIG_SMP
  /* In the SMP case, the thread may be running on another CPU.  If that is
   * the case, then we will pause the CPU that the thread is running on.
   */

  cpu = nxsched_pause_cpu(dtcb);

  /* Get the task list associated with the thread's state and CPU */

  tasklist = TLIST_HEAD(dtcb->task_state, cpu);
#else
  /* In the non-SMP case, we can be assured that the task to be terminated
   * is not running.  get the task list associated with the task state.
   */

  tasklist = TLIST_HEAD(dtcb->task_state);
#endif

  /* Remove the task from the task list */

  dq_rem((FAR dq_entry_t *)dtcb, tasklist);
  dtcb->task_state = TSTATE_TASK_INVALID;

  /* At this point, the TCB should no longer be accessible to the system */

#ifdef CONFIG_SMP
  /* Resume the paused CPU (if any) */

  if (cpu >= 0)
    {
      /* I am not yet sure how to handle a failure here. */

      DEBUGVERIFY(up_cpu_resume(cpu));
    }
#endif /* CONFIG_SMP */

  leave_critical_section(flags);

  /* Perform common task termination logic (flushing streams, calling
   * functions registered by at_exit/on_exit, etc.).  We need to do
   * this as early as possible so that higher level clean-up logic
   * can run in a healthy tasking environment.
   *
   * In the case where the task exits via exit(), nxtask_exithook()
   * may be called twice.
   *
   * I suppose EXIT_SUCCESS is an appropriate return value???
   */

  nxtask_exithook(dtcb, EXIT_SUCCESS, nonblocking);

  /* Since all tasks pass through this function as the final step in their
   * exit sequence, this is an appropriate place to inform any instrumentation
   * layer that the task no longer exists.
   */

  sched_note_stop(dtcb);

  /* Deallocate its TCB */

  return nxsched_release_tcb(dtcb, dtcb->flags & TCB_FLAG_TTYPE_MASK);

errout_with_lock:
  leave_critical_section(flags);
  return ret;
}
