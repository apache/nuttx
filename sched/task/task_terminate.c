/****************************************************************************
 * sched/task/task_terminate.c
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <queue.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <arch/irq.h>

#include "sched/sched.h"
#ifndef CONFIG_DISABLE_SIGNALS
# include "signal/signal.h"
#endif
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: task_terminate
 *
 * Description:
 *   This function causes a specified task to cease to exist. Its  stack and
 *   TCB will be deallocated.  This function is the internal implementation
 *   of the task_delete() function.  It includes and additional parameter
 *   to determine if blocking is permitted or not.
 *
 *   This function is the final function called all task termination
 *   sequences.  task_terminate() is called only from task_delete() (with
 *   nonblocking == false) and from task_exit() (with nonblocking == true).
 *
 *   The path through task_exit() supports the final stops of the exit(),
 *   _exit(), and pthread_exit
 *
 *   - pthread_exit().  Calls _exit()
 *   - exit(). Calls _exit()
 *   - _exit().  Calls task_exit() making the currently running task
 *     non-running. task_exit then calls task_terminate() (with nonblocking
 *     == true) to terminate the non-running task.
 *
 *   NOTE: that the state of non-blocking is irrelevant when called through
 *   exit() and pthread_exit().  In those cases task_exithook() has already
 *   been called with nonblocking == false;
 *
 * Inputs:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *   nonblocking - True: The task is an unhealthy, partially torn down
 *         state and is not permitted to block.
 *
 * Return Value:
 *   OK on success; or ERROR on failure
 *
 *   This function can fail if the provided pid does not correspond to a
 *   task (errno is not set)
 *
 ****************************************************************************/

int task_terminate(pid_t pid, bool nonblocking)
{
  FAR struct tcb_s *dtcb;
  irqstate_t saved_state;

  /* Make sure the task does not become ready-to-run while we are futzing with
   * its TCB by locking ourselves as the executing task.
   */

  sched_lock();

  /* Find for the TCB associated with matching PID */

  dtcb = sched_gettcb(pid);
  if (!dtcb)
    {
      /* This PID does not correspond to any known task */

      sched_unlock();
      return -ESRCH;
    }

  /* Verify our internal sanity */

  if (dtcb->task_state == TSTATE_TASK_RUNNING ||
      dtcb->task_state >= NUM_TASK_STATES)
    {
      sched_unlock();
      PANIC();
    }

  /* Perform common task termination logic (flushing streams, calling
   * functions registered by at_exit/on_exit, etc.).  We need to do
   * this as early as possible so that higher level clean-up logic
   * can run in a healthy tasking environment.
   *
   * In the case where the task exits via exit(), task_exithook()
   * may be called twice.
   *
   * I suppose EXIT_SUCCESS is an appropriate return value???
   */

  task_exithook(dtcb, EXIT_SUCCESS, nonblocking);

  /* Remove the task from the OS's tasks lists. */

  saved_state = irqsave();
  dq_rem((FAR dq_entry_t*)dtcb, (dq_queue_t*)g_tasklisttable[dtcb->task_state].list);
  dtcb->task_state = TSTATE_TASK_INVALID;
  irqrestore(saved_state);

  /* At this point, the TCB should no longer be accessible to the system */

  sched_unlock();

  /* Since all tasks pass through this function as the final step in their
   * exit sequence, this is an appropriate place to inform any instrumentation
   * layer that the task no longer exists.
   */

  sched_note_stop(dtcb);

  /* Deallocate its TCB */

  return sched_releasetcb(dtcb, dtcb->flags & TCB_FLAG_TTYPE_MASK);
}
