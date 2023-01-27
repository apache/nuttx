/****************************************************************************
 * sched/task/task_getpid.c
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
 * Name: getpid
 *
 * Description:
 *   Get the Process ID of the currently executing task.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   Normally when called from user applications, getpid() will return the
 *   Process ID of the currently executing task. that is, the main task
 *   for the task groups. There is no specification for any errors
 *   returned from getpid().
 *
 ****************************************************************************/

pid_t getpid(void)
{
  FAR struct tcb_s *rtcb;

  /* Get the TCB at the head of the ready-to-run task list.  That
   * will usually be the currently executing task.  There is are two
   * exceptions to this:
   *
   * Early in the start-up sequence, the ready-to-run list may be
   * empty!  In this case, of course, the CPU0 start-up/IDLE thread with
   * pid == 0 must be running, and
   */

  rtcb = this_task();
  if (rtcb != NULL)
    {
      /* Yes.. Return the Process ID */

      return rtcb->group->tg_pid;
    }

  /* We must have been called earlier in the start up sequence from the
   * start-up/IDLE thread before the ready-to-run list has been initialized.
   */

  return IDLE_PROCESS_ID;
}
