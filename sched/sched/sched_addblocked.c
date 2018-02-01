/****************************************************************************
 * sched/sched/sched_addblocked.c
 *
 *   Copyright (C) 2007, 2009, 2016 Gregory Nutt. All rights reserved.
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

#include <queue.h>
#include <assert.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_addblocked
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

void sched_addblocked(FAR struct tcb_s *btcb, tstate_t task_state)
{
  FAR dq_queue_t *tasklist;

  /* Make sure that we received a valid blocked state */

  DEBUGASSERT(task_state >= FIRST_BLOCKED_STATE &&
              task_state <= LAST_BLOCKED_STATE);

#ifdef CONFIG_SMP
  /* Lock the tasklists before accessing */

  irqstate_t lock = sched_tasklist_lock();
#endif

  /* Add the TCB to the blocked task list associated with this state. */

  tasklist = TLIST_BLOCKED(task_state);

  /* Determine if the task is to be added to a prioritized task list. */

  if (TLIST_ISPRIORITIZED(task_state))
    {
      /* Add the task to a prioritized list */

      sched_addprioritized(btcb, tasklist);
    }
  else
    {
      /* Add the task to a non-prioritized list */

      dq_addlast((FAR dq_entry_t *)btcb, tasklist);
    }

#ifdef CONFIG_SMP
  /* Unlock the tasklists */

  sched_tasklist_unlock(lock);
#endif

  /* Make sure the TCB's state corresponds to the list */

  btcb->task_state = task_state;
}
