/****************************************************************************
 * sched/task/task_gettid.c
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

#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "sched/sched.h"
#include "task/task.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gettid
 *
 * Description:
 *   Get the thread ID of the currently executing thread.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   On success, returns the thread ID of the calling process.
 *
 ****************************************************************************/

pid_t gettid(void)
{
  FAR struct tcb_s *rtcb;

  /* Get the TCB at the head of the ready-to-run task list.  That
   * will usually be the currently executing task.  There are two
   * exceptions to this:
   *
   * 1. Early in the start-up sequence, the ready-to-run list may be
   *    empty!  In this case, of course, the CPU0 start-up/IDLE thread with
   *    pid == 0 must be running, and
   * 2. As described above, during certain context-switching conditions the
   *    task at the head of the ready-to-run list may not actually be
   *    running.
   */

  rtcb = this_task();
  if (rtcb != NULL)
    {
      /* Check if the task is actually running */

      if (rtcb->task_state == TSTATE_TASK_RUNNING)
        {
          /* Yes.. Return the task ID from the TCB at the head of the
           * ready-to-run task list
           */

          return rtcb->pid;
        }

      /* No.. return -ESRCH to indicate this condition */

      return (pid_t)-ESRCH;
    }

  /* We must have been called earlier in the start up sequence from the
   * start-up/IDLE thread before the ready-to-run list has been initialized.
   */

  return 0;
}
