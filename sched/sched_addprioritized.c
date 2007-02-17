/************************************************************
 * sched_addprioritized.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <queue.h>
#include <assert.h>
#include "os_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function: sched_addprioritized
 *
 * Description:
 *  This function adds a TCB to a prioritized TCB list.
 *
 * Inputs:
 *   tcb - Points to the TCB to add to the prioritized list
 *   list - Points to the prioritized list to add tcb to
 *
 * Return Value:
 *   TRUE if the head of the list has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before
 *   calling this function (calling sched_lock() first is NOT
 *   a good idea -- use irqsave()).
 * - The caller has already removed the input tcb from
 *   whatever list it was in.
 * - The caller handles the condition that occurs if the
 *   the head of the task list is changed.
 * - The caller must set the task_state field of the TCB to
 *   match the state associated with the list.
 ************************************************************/

boolean sched_addprioritized(_TCB *tcb, dq_queue_t *list)
{
  _TCB *next;
  _TCB *prev;
  ubyte sched_priority = tcb->sched_priority;
  boolean ret = FALSE;

  /* Lets do a sanity check before we get started. */

  ASSERT(sched_priority >= SCHED_PRIORITY_MIN);

  /* Search the list to find the location to insert the new Tcb.
   * Each is list is maintained in ascending sched_priority order.
   */

  for (next = (_TCB*)list->head;
      (next && sched_priority <= next->sched_priority);
      next = next->flink);

  /* Add the tcb to the spot found in the list.  Check if the tcb 
   * goes at the end of the list. NOTE:  This could only happen if list
   * is the g_pendingtasks list!
   */

  if (!next)
    {
      /* The tcb goes at the end of the list. */

      prev = (_TCB*)list->tail;
      if (!prev)
        {
          /* Special case:  The list is empty */

          tcb->flink = NULL;
          tcb->blink = NULL;
          list->head = (dq_entry_t*)tcb;
          list->tail = (dq_entry_t*)tcb;
          ret = TRUE;
        }
      else
        {
          /* The tcb goes at the end of a non-empty list */

          tcb->flink = NULL;
          tcb->blink = prev;
          prev->flink = tcb;
          list->tail = (dq_entry_t*)tcb;
        }
    }
  else
    {
      /* The tcb goes just before next */

      prev = (_TCB*)next->blink;
      if (!prev)
        {
          /* Special case:  Insert at the head of the list */

          tcb->flink = next;
          tcb->blink = NULL;
          next->blink = tcb;
          list->head = (dq_entry_t*)tcb;
          ret = TRUE;
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
