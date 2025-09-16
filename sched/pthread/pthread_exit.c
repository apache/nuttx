/****************************************************************************
 * sched/pthread/pthread_exit.c
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

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "task/task.h"
#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_exit
 *
 * Description:
 *   Terminate execution of a thread started with pthread_create.
 *
 * Input Parameters:
 *   exit_value
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nx_pthread_exit(FAR void *exit_value)
{
  FAR struct tcb_s *tcb = this_task();
  irqstate_t flags;
  bool exiting = false;
#ifndef CONFIG_DISABLE_ALL_SIGNALS
  sigset_t set;
#endif
  int status;

  sinfo("exit_value=%p\n", exit_value);

  DEBUGASSERT(tcb != NULL);

  /* Block any signal actions that would awaken us while were
   * are performing the JOIN handshake.
   */

#ifndef CONFIG_DISABLE_ALL_SIGNALS
  sigfillset(&set);
  nxsig_procmask(SIG_SETMASK, &set, NULL);
#endif

  /* Complete pending join operations */

  status = pthread_completejoin(nxsched_gettid(), exit_value);
  if (status != OK)
    {
      /* Assume that the join completion failed because this is
       * not really a pthread.  Exit by calling exit().
       */

      _exit(EXIT_FAILURE);
    }

  /* Make sure that we are in a critical section with local interrupts.
   * The IRQ state will be restored when the next task is started.
   */

  flags = enter_critical_section();

  /* Perform common task termination logic.  This will get called again later
   * through logic kicked off by up_exit().
   *
   * REVISIT: Tt should not be necessary to call this here, but releasing the
   * task group (especially the group file list) requires that it is done
   * here.
   *
   * The reason? up_exit removes the current process from the ready-to-run
   * list and trying to execute code that depends on this_task() crashes at
   * once, or does something very naughty.
   */

  if (tcb->flags & TCB_FLAG_EXIT_PROCESSING)
    {
      exiting = true;
    }
  else
    {
      tcb->flags |= TCB_FLAG_EXIT_PROCESSING;
    }

  leave_critical_section(flags);

  if (exiting)
    {
      /* If the TCB is already in the exiting state, we
       * should allow the killing task to execute normally first.
       * We stop the execution here.
       */

      for (; ; )
        {
          usleep(1000);
        }
    }

  enter_critical_section();

  nxtask_exithook(tcb, status);

  /* In nxtask_exithook, nxmutex_lock is used, and nxmutex_lock depends
   * on nxsched_get_tcb. Therefore, we move nxsched_release_pid
   * to this position.
   */

  nxsched_release_pid(tcb->pid);
  up_exit(EXIT_SUCCESS);
}
