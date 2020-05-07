/****************************************************************************
 * sched/mqueue/mq_waitirq.c
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

#include <sched.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mqueue.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_wait_irq
 *
 * Description:
 *   This function is called when a signal or a timeout is received by a
 *   task that is waiting on a message queue -- either for a queue to
 *   becoming not full (on mq_send and friends) or not empty (on mq_receive
 *   and friends).
 *
 * Input Parameters:
 *   wtcb - A pointer to the TCB of the task that is waiting on a message
 *          queue, but has received a signal instead.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxmq_wait_irq(FAR struct tcb_s *wtcb, int errcode)
{
  FAR struct mqueue_inode_s *msgq;
  irqstate_t flags;

  /* Disable interrupts.  This is necessary because an interrupt handler may
   * attempt to send a message while we are doing this.
   */

  flags = enter_critical_section();

  /* It is possible that an interrupt/context switch beat us to the punch and
   * already changed the task's state.  NOTE:  The operations within the if
   * are safe because interrupts are always disabled with the msgwaitq,
   * nwaitnotempty, and nwaitnotfull fields are modified.
   */

  if (wtcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
      wtcb->task_state == TSTATE_WAIT_MQNOTFULL)
    {
      /* Get the message queue associated with the waiter from the TCB */

      msgq = wtcb->msgwaitq;
      DEBUGASSERT(msgq);

      wtcb->msgwaitq = NULL;

      /* Decrement the count of waiters and cancel the wait */

      if (wtcb->task_state == TSTATE_WAIT_MQNOTEMPTY)
        {
          DEBUGASSERT(msgq->nwaitnotempty > 0);
          msgq->nwaitnotempty--;
        }
      else
        {
          DEBUGASSERT(msgq->nwaitnotfull > 0);
          msgq->nwaitnotfull--;
        }

      /* Mark the error value for the thread. */

      wtcb->errcode = errcode;

      /* Restart the task. */

      up_unblock_task(wtcb);
    }

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);
}
