/****************************************************************************
 * sched/task/task_starthook.c
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

#include <assert.h>
#include <nuttx/sched.h>

#include "task/task.h"

#ifdef CONFIG_SCHED_STARTHOOK

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_starthook
 *
 * Description:
 *   Configure a start hook... a function that will be called on the thread
 *   of the new task before the new task's main entry point is called.
 *   The start hook is useful, for example, for setting up automatic
 *   configuration of C++ constructors.
 *
 * Input Parameters:
 *   tcb - The new, unstarted task task that needs the start hook
 *   starthook - The pointer to the start hook function
 *   arg - The argument to pass to the start hook function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_starthook(FAR struct task_tcb_s *tcb, starthook_t starthook,
                      FAR void *arg)
{
  /* Only tasks can have starthooks.  The starthook will be called when the
   * task is started (or restarted).
   */

#ifndef CONFIG_DISABLE_PTHREAD
  DEBUGASSERT(tcb &&
              (tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) !=
               TCB_FLAG_TTYPE_PTHREAD);
#endif

  /* Set up the start hook */

  tcb->starthook    = starthook;
  tcb->starthookarg = arg;
}

#endif /* CONFIG_SCHED_STARTHOOK */
