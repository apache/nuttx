/****************************************************************************
 * sched/task/task_delete.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_delete
 *
 * Description:
 *   This function causes a specified task to cease to exist.  Its stack and
 *   TCB will be deallocated.  This function is the companion to
 *   task_create().  This is the version of the function exposed to the
 *   user; it is simply a wrapper around the internal, nxtask_terminate
 *   function.
 *
 *   The logic in this function only deletes non-running tasks.  If the
 *   'pid' parameter refers to the currently running task, then processing
 *   is redirected to exit().  This can only happen if a task calls
 *   task_delete()in order to delete itself.
 *
 *   This function obeys the semantics of pthread cancellation:  task
 *   deletion is deferred if cancellation is disabled or if deferred
 *   cancellation is supported (with cancellation points enabled).
 *
 * Input Parameters:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *
 * Returned Value:
 *   OK on success; or ERROR on failure with the errno variable set
 *   appropriately.
 *
 ****************************************************************************/

int task_delete(pid_t pid)
{
  FAR struct tcb_s *dtcb;
  FAR struct tcb_s *rtcb;
  int errcode;
  int ret;

  /* Check if the task to delete is the calling task:  PID=0 means to delete
   * the calling task.  In this case, task_delete() is much like exit()
   * except that it obeys the cancellation semantics.
   */

  rtcb = this_task();
  if (pid == 0)
    {
      pid = rtcb->pid;
    }

  /* Get the TCB of the task to be deleted */

  dtcb = (FAR struct tcb_s *)nxsched_get_tcb(pid);
  if (dtcb == NULL)
    {
      /* The pid does not correspond to any known thread.  The task
       * has probably already exited.
       */

      errcode = ESRCH;
      goto errout;
    }

  /* Only tasks and kernel threads can be deleted with this interface
   * (The semantics of the call should be sufficient to prohibit this).
   */

  DEBUGASSERT((dtcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD);

  /* Non-privileged tasks and pthreads may not delete privileged kernel
   * threads.
   *
   * REVISIT: We will need to look at this again in the future if/when
   * permissions are supported and a user task might also be privileged.
   */

  if (((rtcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL) &&
      ((dtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL))
    {
      errcode = EACCES;
      goto errout;
    }

  /* Check if the task to delete is the calling task */

  if (pid == rtcb->pid)
    {
      /* If it is, then what we really wanted to do was exit. Note that we
       * don't bother to unlock the TCB since it will be going away.
       */

      exit(EXIT_SUCCESS);
    }

  /* Notify the target if the non-cancelable or deferred cancellation set */

  if (nxnotify_cancellation(dtcb))
    {
      return OK;
    }

  /* Otherwise, perform the asynchronous cancellation, letting
   * nxtask_terminate() do all of the heavy lifting.
   */

  ret = nxtask_terminate(pid, false);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }

  return OK;

errout:
  set_errno(errcode);
  return ERROR;
}
