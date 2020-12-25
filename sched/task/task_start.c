/****************************************************************************
 * sched/task/task_start.c
 *
 *   Copyright (C) 2007-2010, 2013, 2018 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "group/group.h"
#include "sched/sched.h"
#include "signal/signal.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_START_ARGS 256

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_start
 *
 * Description:
 *   This function is the low level entry point into the main thread of
 *   execution of a task.  It receives initial control when the task is
 *   started and calls main entry point of the newly started task.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_start(void)
{
  FAR struct task_tcb_s *tcb = (FAR struct task_tcb_s *)this_task();
  int exitcode = EXIT_FAILURE;
  int argc;

  DEBUGASSERT((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) != \
              TCB_FLAG_TTYPE_PTHREAD);

#ifdef CONFIG_SIG_DEFAULT
  /* Set up default signal actions */

  nxsig_default_initialize(&tcb->cmn);
#endif

  /* Execute the start hook if one has been registered */

#ifdef CONFIG_SCHED_STARTHOOK
  if (tcb->starthook != NULL)
    {
      tcb->starthook(tcb->starthookarg);
    }
#endif

  /* Count how many non-null arguments we are passing. The first non-null
   * argument terminates the list .
   */

  argc = 1;
  while (tcb->argv[argc])
    {
      /* Increment the number of args.  Here is a sanity check to
       * prevent running away with an unterminated argv[] list.
       * MAX_START_ARGS should be sufficiently large that this never
       * happens in normal usage.
       */

      if (++argc > MAX_START_ARGS)
        {
          exit(EXIT_FAILURE);
        }
    }

  /* Call the 'main' entry point passing argc and argv.  In the kernel build
   * this has to be handled differently if we are starting a user-space task;
   * we have to switch to user-mode before calling the task.
   */

  if ((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      exitcode = tcb->cmn.entry.main(argc, tcb->argv);
    }
  else
    {
#ifdef CONFIG_BUILD_FLAT
      nxtask_startup(tcb->cmn.entry.main, argc, tcb->argv);
#else
      up_task_start(tcb->cmn.entry.main, argc, tcb->argv);
#endif
    }

  /* Call exit() if/when the task returns */

  exit(exitcode);
}
