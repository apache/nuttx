/****************************************************************************
 * sched/sched/sched_addblocked.c
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
 * Name: nxsched_add_blocked
 *
 * Description:
 *   This function adds a TCB to one of the blocked state task lists as
 *   inferred from task_state.
 *
 * Input Parameters:
 *   btcb - Points to the TCB that is blocked
 *   task_state - identifies the state of the blocked task
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function.
 *
 ****************************************************************************/

void nxsched_add_blocked(FAR struct tcb_s *btcb, tstate_t task_state)
{
  FAR dq_queue_t *tasklist;

  /* Make sure that we received a valid blocked state */

  DEBUGASSERT(task_state >= FIRST_BLOCKED_STATE &&
              task_state <= LAST_BLOCKED_STATE);

  /* Make sure the TCB's state corresponds to the list */

  btcb->task_state = task_state;

  /* Add the TCB to the blocked task list associated with this state. */

  tasklist = TLIST_BLOCKED(btcb);

  /* Determine if the task is to be added to a prioritized task list. */

  if (TLIST_ISPRIORITIZED(task_state))
    {
      /* Add the task to a prioritized list */

      nxsched_add_prioritized(btcb, tasklist);
    }
  else
    {
      /* Add the task to a non-prioritized list */

      dq_addlast((FAR dq_entry_t *)btcb, tasklist);
    }
}
