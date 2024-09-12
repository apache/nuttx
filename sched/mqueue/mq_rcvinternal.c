/****************************************************************************
 * sched/mqueue/mq_rcvinternal.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <mqueue.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_rcvtimeout
 *
 * Description:
 *   This function is called if the timeout elapses before the message queue
 *   becomes non-empty.
 *
 * Input Parameters:
 *   arg - the argument provided when the timeout was configured.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void nxmq_rcvtimeout(wdparm_t arg)
{
  FAR struct tcb_s *wtcb = (FAR struct tcb_s *)(uintptr_t)arg;
  irqstate_t flags;

  /* Disable interrupts.  This is necessary because an interrupt handler may
   * attempt to send a message while we are doing this.
   */

  flags = enter_critical_section();

  /* It is also possible that an interrupt/context switch beat us to the
   * punch and already changed the task's state.
   */

  if (wtcb->task_state == TSTATE_WAIT_MQNOTEMPTY)
    {
      /* Restart with task with a timeout error */

      nxmq_wait_irq(wtcb, ETIMEDOUT);
    }

  /* Interrupts may now be re-enabled. */

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_wait_receive
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_receive and
 *   [nx]mq_timedreceive.  This function waits for a message to be received
 *   on the specified message queue, removes the message from the queue, and
 *   returns it.
 *
 * Input Parameters:
 *   msgq   - Message queue descriptor
 *   rcvmsg - The caller-provided location in which to return the newly
 *            received message.
 *   abstime - If non-NULL, this is the absolute time to wait until a
 *             message is received.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  A negated errno value is returned
 *   on any failure.
 *
 * Assumptions:
 * - The caller has provided all validity checking of the input parameters
 *   using nxmq_verify_receive.
 * - Interrupts should be disabled throughout this call.  This is necessary
 *   because messages can be sent from interrupt level processing.
 * - For mq_timedreceive, setting of the timer and this wait must be atomic.
 *
 ****************************************************************************/

int nxmq_wait_receive(FAR struct mqueue_inode_s *msgq,
                      FAR struct mqueue_msg_s **rcvmsg,
                      FAR const struct timespec *abstime,
                      sclock_t ticks)
{
  FAR struct mqueue_msg_s *newmsg;
  FAR struct tcb_s *rtcb = this_task();

#ifdef CONFIG_CANCELLATION_POINTS
  /* nxmq_wait_receive() is not a cancellation point, but it may be called
   * from mq_receive() or mq_timedreceive() which are cancellation point.
   */

  if (check_cancellation_point())
    {
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      return -ECANCELED;
    }
#endif

  if (abstime)
    {
      wd_start_realtime(&rtcb->waitdog, abstime,
                        nxmq_rcvtimeout, (wdparm_t)rtcb);
    }
  else if (ticks >= 0)
    {
      wd_start(&rtcb->waitdog, ticks,
               nxmq_rcvtimeout, (wdparm_t)rtcb);
    }

  /* Get the message from the head of the queue */

  while ((newmsg = (FAR struct mqueue_msg_s *)
                   list_remove_head(&msgq->msglist)) == NULL)
    {
      msgq->cmn.nwaitnotempty++;

      /* Initialize the 'errcode" used to communication wake-up error
       * conditions.
       */

      rtcb->waitobj = msgq;
      rtcb->errcode = OK;

      /* Remove the tcb task from the running list. */

      nxsched_remove_self(rtcb);

      /* Add the task to the specified blocked task list */

      rtcb->task_state = TSTATE_WAIT_MQNOTEMPTY;
      nxsched_add_prioritized(rtcb, MQ_WNELIST(msgq->cmn));

      /* Now, perform the context switch */

      up_switch_context(this_task(), rtcb);

      /* When we resume at this point, either (1) the message queue
       * is no longer empty, or (2) the wait has been interrupted by
       * a signal.  We can detect the latter case be examining the
       * errno value (should be either EINTR or ETIMEDOUT).
       */

      if (rtcb->errcode != OK)
        {
          break;
        }
    }

  if (abstime || ticks >= 0)
    {
      wd_cancel(&rtcb->waitdog);
    }

  *rcvmsg = newmsg;
  return -rtcb->errcode;
}

/****************************************************************************
 * Name: nxmq_notify_receive
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_receive and
 *   [nx]mq_timedreceive.
 *   This function notifies any tasks that are waiting for the message queue
 *   to become non-empty. This function is called after a message is
 *   received from the message queue.
 *
 * Input Parameters:
 *   msgq    - Message queue descriptor
 *
 * Returned Value:
 *   Returns the length of the received message.  This function does not
 *   fail.
 *
 * Assumptions:
 * - Pre-emption should be disabled throughout this call.
 *
 ****************************************************************************/

void nxmq_notify_receive(FAR struct mqueue_inode_s *msgq)
{
  FAR struct tcb_s *btcb;

  /* Check if any tasks are waiting for the MQ not full event. */

  if (msgq->cmn.nwaitnotfull > 0)
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Find the highest priority task that is waiting for
       * this queue to be not-full in waitfornotfull list.
       * This must be performed in a critical section because
       * messages can be sent from interrupt handlers.
       */

      btcb = (FAR struct tcb_s *)dq_remfirst(MQ_WNFLIST(msgq->cmn));

      /* If one was found, unblock it.  NOTE:  There is a race
       * condition here:  the queue might be full again by the
       * time the task is unblocked
       */

      DEBUGASSERT(btcb != NULL);

      if (WDOG_ISACTIVE(&btcb->waitdog))
        {
          wd_cancel(&btcb->waitdog);
        }

      msgq->cmn.nwaitnotfull--;

      /* Indicate that the wait is over. */

      btcb->waitobj = NULL;

      /* Add the task to ready-to-run task list and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(btcb))
        {
          up_switch_context(btcb, rtcb);
        }
    }
}
