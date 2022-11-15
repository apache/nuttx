/****************************************************************************
 * sched/sched/sched_reprioritizertr.c
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
#include <sched.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_reprioritize_rtr
 *
 * Description:
 *   This function called when the priority of a running or
 *   ready-to-run task changes.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   priority - The new task priority
 *
 * Returned Value:
 *   true if the head of the ready-to-run task list has changed indicating
 *     a context switch is needed.
 *
 ****************************************************************************/

bool nxsched_reprioritize_rtr(FAR struct tcb_s *tcb, int priority)
{
  bool switch_needed;

  /* Remove the tcb task from the ready-to-run list.
   * nxsched_remove_readytorun will return true if we just
   * remove the head of the ready to run list.
   */

  switch_needed = nxsched_remove_readytorun(tcb, false);

  /* Setup up the new task priority */

  tcb->sched_priority = (uint8_t)priority;

  /* Return the task to the specified blocked task list.
   * nxsched_add_readytorun will return true if the task was
   * added to the new list.  We will need to perform a context
   * switch only if the EXCLUSIVE or of the two calls is non-zero
   * (i.e., one and only one the calls changes the head of the
   * ready-to-run list).
   */

  switch_needed ^= nxsched_add_readytorun(tcb);

  /* If we are going to do a context switch, then now is the right
   * time to add any pending tasks back into the ready-to-run list.
   */

  if (switch_needed && g_pendingtasks.head)
    {
      nxsched_merge_pending();
    }

  return switch_needed;
}
