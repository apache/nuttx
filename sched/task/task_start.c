/****************************************************************************
 * sched/task/task_start.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#include "group/group.h"
#include "sched/sched.h"
#include "signal/signal.h"
#include "task/task.h"

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
  FAR struct tcb_s *tcb = this_task();
  uint8_t ttype = tcb->flags & TCB_FLAG_TTYPE_MASK;
#ifdef CONFIG_SCHED_STARTHOOK
  FAR struct task_tcb_s *ttcb = (FAR struct task_tcb_s *)tcb;
#endif
  int exitcode = EXIT_FAILURE;
  FAR char **argv;
  int argc;

  DEBUGASSERT(ttype != TCB_FLAG_TTYPE_PTHREAD);

#ifdef CONFIG_SIG_DEFAULT
  if (ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      /* Set up default signal actions for NON-kernel thread */

      nxsig_default_initialize(tcb);
    }
#endif

  /* Execute the start hook if one has been registered */

#ifdef CONFIG_SCHED_STARTHOOK
  if (ttype != TCB_FLAG_TTYPE_KERNEL && ttcb->starthook != NULL)
    {
      ttcb->starthook(ttcb->starthookarg);
    }
#endif

  /* Take args from stack, as group is shared for kthreads */

  argv = nxsched_get_stackargs(tcb);
  for (argc = 0; argv && argv[argc]; argc++);

  /* Call the 'main' entry point passing argc and argv.  In the kernel build
   * this has to be handled differently if we are starting a user-space task;
   * we have to switch to user-mode before calling the task.
   */

  if (ttype == TCB_FLAG_TTYPE_KERNEL)
    {
      exitcode = tcb->entry.main(argc, argv);
    }
  else
    {
#ifdef CONFIG_BUILD_FLAT
      nxtask_startup(tcb->entry.main, argc, argv);
#else
      up_task_start(tcb->entry.main, argc, argv);
#endif
    }

  /* Call _exit() if/when the task returns */

  _exit(exitcode);
}
