/****************************************************************************
 * sched/task/task_getppid.c
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
 * Name: getppid
 *
 * Description:
 *   Get the parent task ID of the currently executing task.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   Normally when called from user applications, getppid() will return the
 *   parent task ID of the currently executing task, that is, the task at the
 *   head of the ready-to-run list.  There is no specification for any errors
 *   returned from getppid().
 *
 *   getppid(), however, may be called from within the OS in some cases.
 *   There are certain situations during context switching when the OS data
 *   structures are in flux and where the current task at the head of the
 *   ready-to-run task list is not actually running.  In that case,
 *   getppid() will return the error: -ESRCH
 *
 ****************************************************************************/

pid_t getppid(void)
{
  FAR struct tcb_s *rtcb;

  /* Get the TCB at the head of the ready-to-run task list.  That
   * will usually be the currently executing task.  There is are two
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
          /* Yes.. Return the parent task ID from the TCB at the head of the
           * ready-to-run task list
           */

          return rtcb->group->tg_ppid;
        }

      /* No.. return -ESRCH to indicate this condition */

      return (pid_t)-ESRCH;
    }

  /* We must have been called earlier in the start up sequence from the
   * start-up/IDLE thread before the ready-to-run list has been initialized.
   */

  return IDLE_PROCESS_ID;
}
