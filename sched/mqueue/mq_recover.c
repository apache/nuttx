/****************************************************************************
 *  sched/mqueue/mq_recover.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/mqueue.h>
#include <nuttx/sched.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_recover
 *
 * Description:
 *   This function is called when a task is deleted via task_deleted or
 *   via pthread_cancel. I checks if the task was waiting for a message
 *   queue event and adjusts counts appropriately.
 *
 * Input Parameters:
 *   tcb - The TCB of the terminated task or thread
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

void nxmq_recover(FAR struct tcb_s *tcb)
{
  /* If were were waiting for a timed message queue event, then the
   * timer was canceled and deleted in task_recover() before this
   * function was called.
   */

  /* Was the task waiting for a message queue to become non-empty? */

  if (tcb->task_state == TSTATE_WAIT_MQNOTEMPTY)
    {
      /* Decrement the count of waiters */

      DEBUGASSERT(tcb->msgwaitq && tcb->msgwaitq->nwaitnotempty > 0);
      tcb->msgwaitq->nwaitnotempty--;
    }

  /* Was the task waiting for a message queue to become non-full? */

  else if (tcb->task_state == TSTATE_WAIT_MQNOTFULL)
    {
      /* Decrement the count of waiters */

      DEBUGASSERT(tcb->msgwaitq && tcb->msgwaitq->nwaitnotfull > 0);
      tcb->msgwaitq->nwaitnotfull--;
    }
}
