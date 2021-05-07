/****************************************************************************
 * sched/sched/sched_addprioritized.c
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
#include <stdbool.h>
#include <queue.h>
#include <assert.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_add_prioritized
 *
 * Description:
 *  This function adds a TCB to a prioritized TCB list.
 *
 * Input Parameters:
 *   tcb - Points to the TCB to add to the prioritized list
 *   list - Points to the prioritized list to add tcb to
 *
 * Returned Value:
 *   true if the head of the list has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function (calling sched_lock() first is NOT
 *   a good idea -- use enter_critical_section()).
 * - The caller has already removed the input tcb from
 *   whatever list it was in.
 * - The caller handles the condition that occurs if the
 *   the head of the task list is changed.
 * - The caller must set the task_state field of the TCB to
 *   match the state associated with the list.
 *
 ****************************************************************************/

bool nxsched_add_prioritized(FAR struct tcb_s *tcb, DSEG dq_queue_t *list)
{
  FAR struct tcb_s *next;
  FAR struct tcb_s *prev;
  uint8_t sched_priority = tcb->sched_priority;
  bool ret = false;

  /* Lets do a sanity check before we get started. */

  DEBUGASSERT(sched_priority >= SCHED_PRIORITY_MIN);

  /* Search the list to find the location to insert the new Tcb.
   * Each is list is maintained in descending sched_priority order.
   */

  for (next = (FAR struct tcb_s *)list->head;
       (next && sched_priority <= next->sched_priority);
       next = next->flink);

  /* Add the tcb to the spot found in the list.  Check if the tcb
   * goes at the end of the list. NOTE:  This could only happen if list
   * is the g_pendingtasks list!
   */

  if (!next)
    {
      /* The tcb goes at the end of the list. */

      prev = (FAR struct tcb_s *)list->tail;
      if (!prev)
        {
          /* Special case:  The list is empty */

          tcb->flink = NULL;
          tcb->blink = NULL;
          list->head = (FAR dq_entry_t *)tcb;
          list->tail = (FAR dq_entry_t *)tcb;
          ret = true;
        }
      else
        {
          /* The tcb goes at the end of a non-empty list */

          tcb->flink = NULL;
          tcb->blink = prev;
          prev->flink = tcb;
          list->tail = (FAR dq_entry_t *)tcb;
        }
    }
  else
    {
      /* The tcb goes just before next */

      prev = (FAR struct tcb_s *)next->blink;
      if (!prev)
        {
          /* Special case:  Insert at the head of the list */

          tcb->flink  = next;
          tcb->blink  = NULL;
          next->blink = tcb;
          list->head  = (FAR dq_entry_t *)tcb;
          ret = true;
        }
      else
        {
          /* Insert in the middle of the list */

          tcb->flink = next;
          tcb->blink = prev;
          prev->flink = tcb;
          next->blink = tcb;
        }
    }

  return ret;
}
