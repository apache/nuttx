/****************************************************************************
 * sched/sched/sched_mergeprioritized.c
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
 * Name: nxsched_merge_prioritized
 *
 * Description:
 *  This function merges the content of the prioritized task list '1ist1'
 *  into the prioritized task list, 'list2'.  On return 'list2' will contain
 *  the prioritized content of both lists; 'list1' will be empty.
 *
 * Input Parameters:
 *   list1 - Points to the prioritized list to merge into list 1.  This list
 *           will be empty upon return.
 *   list2 - That list that will contained the prioritized content of
 *           both lists upon return.
 *   task_state - The task state/list index associated with list2.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 *
 ****************************************************************************/

void nxsched_merge_prioritized(FAR dq_queue_t *list1, FAR dq_queue_t *list2,
                               uint8_t task_state)
{
  dq_queue_t clone;
  FAR struct tcb_s *tcb1;
  FAR struct tcb_s *tcb2;
  FAR struct tcb_s *tmp;

#ifdef CONFIG_SMP
  /* Lock the tasklists before accessing */

  irqstate_t lock = nxsched_lock_tasklist();
#endif

  DEBUGASSERT(list1 != NULL && list2 != NULL);

  /* Get a private copy of list1, clearing list1.  We do this early so that
   * we can be assured that the list is stationary before we start any
   * operations on it.
   */

  dq_move(list1, &clone);

  /* Get the TCB at the head of list1 */

  tcb1 = (FAR struct tcb_s *)dq_peek(&clone);
  if (tcb1 == NULL)
    {
      /* Special case.. list1 is empty.  There is nothing to be done. */

      goto ret_with_lock;
    }

  /* Now the TCBs are no longer accessible and we can change the state on
   * each TCB.  We go through extra precaution to assure that a TCB is never
   * in a list with the wrong state.
   */

  for (tmp  = tcb1;
       tmp != NULL;
       tmp  = (FAR struct tcb_s *)dq_next((FAR dq_entry_t *)tmp))
    {
      tmp->task_state = task_state;
    }

  /* Get the head of list2 */

  tcb2 = (FAR struct tcb_s *)dq_peek(list2);
  if (tcb2 == NULL)
    {
      /* Special case.. list2 is empty.  Move list1 to list2. */

      dq_move(&clone, list2);
      goto ret_with_lock;
    }

  /* Now loop until all entries from list1 have been merged into list2. tcb1
   * points at the TCB at the head of list1; tcb2 points to the TCB at the
   * current working position in list2
   */

  do
    {
      /* Are we at the end of list2 with TCBs remaining to be merged in
       * list1?
       */

      if (tcb2 == NULL)
        {
          /* Yes..  Just append the remainder of list1 to the end of list2. */

          dq_cat(&clone, list2);
          break;
        }

      /* Which TCB has higher priority? */

      else if (tcb1->sched_priority > tcb2->sched_priority)
        {
          /* The TCB from list1 has higher priority than the TCB from list2.
           * Remove the TCB from list1 and insert it before the TCB from
           * list2.
           */

          tmp = (FAR struct tcb_s *)dq_remfirst(&clone);
          DEBUGASSERT(tmp == tcb1);

          dq_addbefore((FAR dq_entry_t *)tcb2, (FAR dq_entry_t *)tmp,
                       list2);

          tcb1 = (FAR struct tcb_s *)dq_peek(&clone);
        }
      else
        {
          /* The TCB from list2 has higher (or same) priority as the TCB
           * from list2.  Skip to the next, lower priority TCB in list2.
           */

          tcb2 = (FAR struct tcb_s *)dq_next((FAR dq_entry_t *)tcb2);
        }
    }
  while (tcb1 != NULL);

ret_with_lock:

#ifdef CONFIG_SMP
  /* Unlock the tasklists */

  nxsched_unlock_tasklist(lock);
#endif
  return;
}
