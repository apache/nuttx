/****************************************************************************
 * sched/sched/sched_mergeprioritized.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <queue.h>
#include <assert.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_mergeprioritized
 *
 * Description:
 *  This function merges the content of the prioritized task list '1ist1'
 *  into the prioritized task list, 'list2'.  On return 'list2' will contain
 *  the prioritized content of both lists; 'list1' will be empty.
 *
 * Inputs:
 *   list1 - Points to the prioritized list to merge into list 1.  This list
 *           will be empty upon return.
 *   list2 - That list that will contained the prioritized content of
 *           both lists upon return.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 *
 ****************************************************************************/

void sched_mergeprioritized(FAR dq_queue_t *list1, FAR dq_queue_t *list2)
{
  FAR struct tcb_s *tcb1;
  FAR struct tcb_s *tcb2;

  DEBUGASSERT(list1 != NULL && list2 != NULL);

  /* Get the head of list1 and list2 */

  tcb1 = (FAR struct tcb_s *)dq_peek(list1);
  if (tcb1 == NULL)
    {
      /* Special case.. list1 is empty.  There is nothing to be done. */

      return;
    }

  tcb2 = (FAR struct tcb_s *)dq_peek(list2);
  if (tcb2 == NULL)
    {
      /* Special case.. list2 is empty.  Move list1 to list2. */

      dq_move(list1, list2);
      return;
    }

  /* Now loop until all entries from list1 have been merged into list2 */

  while (tcb1 != NULL)
    {
      /* Are we at the end of list2? */

      if (tcb2 == NULL)
        {
          /* Yes..  Just append the remainder of list1 to the end of list2. */

          tcb1->blink = NULL;
          list1->head = (FAR dq_entry_t *)tcb1;
          dq_cat(list1, list2);
          break;
        }

      /* Which has higher priority? */

      else if (tcb1->sched_priority > tcb2->sched_priority)
        {
          /* The TCB from list1 has higher priority than the TCB from list2.
           * Insert the TCB from list1 before the TCB from list2.
           */

          dq_addbefore((FAR dq_entry_t *)tcb2, (FAR dq_entry_t *)tcb2,
                       list2);
          tcb1 = (FAR struct tcb_s *)dq_next((FAR dq_entry_t *)tcb1);
        }
      else
        {
          /* The TCB from list2 has higher (or same) priority as the TCB
           * from list2.  Skip to the next, lower priority TCB in list2.
           */

          tcb2 = (FAR struct tcb_s *)dq_next((FAR dq_entry_t *)tcb2);
        }
    }

 /* All of the TCBs from list1 have been moved.  Now we can now mark list1
  * empty.
  */
 
 dq_init(list1);
}

