/*******************************************************************************
 * sched/task/task_terminate.c
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/queue.h>
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
 *   sequences.  nxtask_terminate() is called only from task_delete() and
 *   from nxtask_exit().
 *
 *   The path through nxtask_exit() supports the final stops of the exit(),
 *   _exit(), and pthread_exit
 *
 *   - pthread_exit().  Calls _exit()
 *   - exit(). Calls _exit()
 *   - _exit().  Calls nxtask_exit() making the currently running task
 *     non-running. nxtask_exit then calls nxtask_terminate() to terminate
 *     the non-running task.
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *
 * Returned Value:
 *   OK on success; or ERROR on failure
 *
 *   This function can fail if the provided pid does not correspond to a
 *   task (errno is not set)
 *
 *******************************************************************************/

int nxtask_terminate(pid_t pid)
{
  FAR struct tcb_s *dtcb;
  uint8_t task_state;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Find for the TCB associated with matching PID */

  dtcb = nxsched_get_tcb(pid);
  if (!dtcb)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  /* Remove dtcb from tasklist, let remove_readtorun() do the job */

  task_state = dtcb->task_state;
  nxsched_remove_readytorun(dtcb, false);
  dtcb->task_state = task_state;

  leave_critical_section(flags);

  /* Perform common task termination logic.  We need to do
   * this as early as possible so that higher level clean-up logic
   * can run in a healthy tasking environment.
   *
   * I suppose EXIT_SUCCESS is an appropriate return value???
   */

  nxtask_exithook(dtcb, EXIT_SUCCESS);

  /* Since all tasks pass through this function as the final step in their
   * exit sequence, this is an appropriate place to inform any instrumentation
   * layer that the task no longer exists.
   */

  sched_note_stop(dtcb);

  /* Deallocate its TCB */

  return nxsched_release_tcb(dtcb, dtcb->flags & TCB_FLAG_TTYPE_MASK);
}
