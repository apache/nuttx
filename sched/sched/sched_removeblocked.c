/****************************************************************************
 * sched/sched/sched_removeblocked.c
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

#include <nuttx/queue.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_remove_blocked
 *
 * Description:
 *   This function removes a TCB from one of the blocked state task
 *   lists as inferred from the task_state inside the TCB.
 *
 * Input Parameters:
 *   btcb - Points to the TCB that is blocked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function.
 *
 ****************************************************************************/

void nxsched_remove_blocked(FAR struct tcb_s *btcb)
{
  tstate_t task_state = btcb->task_state;

  /* Make sure the TCB is in a valid blocked state */

  DEBUGASSERT(task_state >= FIRST_BLOCKED_STATE &&
              task_state <= LAST_BLOCKED_STATE);

  /* Remove the TCB from the blocked task list associated
   * with this state
   */

  dq_rem((FAR dq_entry_t *)btcb, TLIST_BLOCKED(btcb));

  /* Indicate that the wait is over. */

  btcb->waitobj = NULL;

  /* Make sure the TCB's state corresponds to not being in
   * any list
   */

  btcb->task_state = TSTATE_TASK_INVALID;
}
