/****************************************************************************
 * sched/task/task_getpid.c
 *
 *   Copyright (C) 2007, 2009, 2014, 2018 Gregory Nutt. All rights reserved.
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
 *   Get the task ID of the currently executing task.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   Normally when called from user applications, getpid() will return the
 *   task ID of the currently executing task, that is, the task at the head
 *   of the ready-to-run list.  There is no specification for any errors
 *   returned from getpid().
 *
 *   getpid(), however, may be called from within the OS in some cases.
 *   There are certain situations during context switching when the OS data
 *   structures are in flux and where the current task at the head of the
 *   ready-to-run task list is not actually running.  In that case,
 *   getpid() will return the error: -ESRCH
 *
 ****************************************************************************/

pid_t getpid(void)
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

  return (pid_t)0;
}
