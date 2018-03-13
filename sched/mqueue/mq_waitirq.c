/****************************************************************************
 * sched/mqueue/mq_waitirq.c
 *
 *   Copyright (C) 2007-2009, 2011, 2014, 2016 Gregory Nutt. All rights reserved.
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

      /* Mark the errno value for the thread. */

      wtcb->pterrno = errcode;

      /* Restart the task. */

      up_unblock_task(wtcb);
    }

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);
}
