/****************************************************************************
 * sched/task/task_delete.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
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
 * Name: task_delete
 *
 * Description:
 *   This function causes a specified task to cease to exist. Its  stack and
 *   TCB will be deallocated.  This function is the companion to task_create().
 *   This is the version of the function exposed to the user; it is simply
 *   a wrapper around the internal, task_terminate function.
 *
 *   The logic in this function only deletes non-running tasks.  If the 'pid'
 *   parameter refers to to the currently runing task, then processing is
 *   redirected to exit().  This can only happen if a task calls task_delete()
 *   in order to delete itself.
 *
 *   In fact, this function (and task_terminate) are the final functions
 *   called all task termination sequences.  task_delete may be called
 *   from:
 *
 *   - task_restart(),
 *   - pthread_cancel(),
 *   - and directly from user code.
 *
 *   Other exit paths (exit(), _eixt(), and pthread_exit()) will go through
 *   task_terminate()
 *
 * Inputs:
 *   pid - The task ID of the task to delete.  A pid of zero
 *         signifies the calling task.
 *
 * Return Value:
 *   OK on success; or ERROR on failure
 *
 *   This function can fail if the provided pid does not correspond to a
 *   task (errno is not set)
 *
 ****************************************************************************/

int task_delete(pid_t pid)
{
  FAR struct tcb_s *rtcb;

  /* Check if the task to delete is the calling task */

  rtcb = (FAR struct tcb_s*)g_readytorun.head;
  if (pid == 0 || pid == rtcb->pid)
    {
      /* If it is, then what we really wanted to do was exit. Note that we
       * don't bother to unlock the TCB since it will be going away.
       */

      exit(EXIT_SUCCESS);
    }

  /* Then let task_terminate do the heavy lifting */

  return task_terminate(pid, false);
}
