/****************************************************************************
 * sched/mqueue/mq_sndinternal.c
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

#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_sndtimeout
 *
 * Description:
 *   This function is called if the timeout elapses before the message queue
 *   becomes non-full.
 *
 * Input Parameters:
 *   arg - The argument that was provided when the timeout was configured.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void nxmq_sndtimeout(wdparm_t arg)
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

  if (wtcb->task_state == TSTATE_WAIT_MQNOTFULL)
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
 * Name: nxmq_wait_send
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_send and
 *   [nx]mq_timesend.  This function waits until the message queue is not
 *   full.
 *
 * Input Parameters:
 *   msgq   - Message queue descriptor
 *   abstime - The absolute time to wait until
 *
 * Returned Value:
 *   On success, nxmq_wait_send() returns 0 (OK); a negated errno value is
 *   returned on any failure:
 *
 *   EAGAIN    The queue was full and the O_NONBLOCK flag was set for the
 *             message queue description referred to by msgq.
 *   EINTR     The call was interrupted by a signal handler.
 *   ETIMEDOUT A timeout expired before the message queue became non-full
 *             (mq_timedsend only).
 *
 * Assumptions/restrictions:
 * - The caller has verified the input parameters using nxmq_verify_send().
 * - Executes within a critical section established by the caller.
 *
 ****************************************************************************/

int nxmq_wait_send(FAR struct mqueue_inode_s *msgq,
                   FAR const struct timespec *abstime,
                   sclock_t ticks)
{
  FAR struct tcb_s *rtcb = this_task();

#ifdef CONFIG_CANCELLATION_POINTS
  /* nxmq_wait_send() is not a cancellation point, but may be called via
   * mq_send() or mq_timedsend() which are cancellation points.
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
                        nxmq_sndtimeout, (wdparm_t)rtcb);
    }
  else if (ticks >= 0)
    {
      wd_start(&rtcb->waitdog, ticks,
               nxmq_sndtimeout, (wdparm_t)rtcb);
    }

  /* Verify that the queue is indeed full as the caller thinks */

  /* Loop until there are fewer than max allowable messages in the
   * receiving message queue
   */

  while (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Block until the message queue is no longer full.
       * When we are unblocked, we will try again
       */

      rtcb          = this_task();
      rtcb->waitobj = msgq;
      msgq->cmn.nwaitnotfull++;

      /* Initialize the errcode used to communication wake-up error
       * conditions.
       */

      rtcb->errcode = OK;

      /* Make sure this is not the idle task, descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(!is_idle_task(rtcb));

      /* Remove the tcb task from the running list. */

      nxsched_remove_self(rtcb);

      /* Add the task to the specified blocked task list */

      rtcb->task_state = TSTATE_WAIT_MQNOTFULL;
      nxsched_add_prioritized(rtcb, MQ_WNFLIST(msgq->cmn));

      /* Now, perform the context switch */

      up_switch_context(this_task(), rtcb);

      /* When we resume at this point, either (1) the message queue
       * is no longer empty, or (2) the wait has been interrupted by
       * a signal.  We can detect the latter case be examining the
       * per-task errno value (should be EINTR or ETIMEDOUT).
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

  return -rtcb->errcode;
}

/****************************************************************************
 * Name: nxmq_notify_send
 *
 * Description:
 *   This function is called when a message is sent to a message queue.
 *   It will notify any tasks that are waiting for the message queue to be
 *   non-full.
 *
 * Input Parameters:
 *   msgq   - Message queue descriptor
 *
 * Returned Value:
 *   None
 *
 * Assumptions/restrictions:
 * - Executes within a critical section established by the caller.
 *
 ****************************************************************************/

void nxmq_notify_send(FAR struct mqueue_inode_s *msgq)
{
  FAR struct tcb_s *btcb;

  /* Check if we need to notify any tasks that are attached to the
   * message queue
   */

#ifndef CONFIG_DISABLE_MQUEUE_NOTIFICATION
  if (msgq->ntpid != INVALID_PROCESS_ID)
    {
      struct sigevent event;
      pid_t pid;

      /* Remove the message notification data from the message queue. */

      memcpy(&event, &msgq->ntevent, sizeof(struct sigevent));
      pid = msgq->ntpid;

      /* Detach the notification */

      memset(&msgq->ntevent, 0, sizeof(struct sigevent));
      msgq->ntpid = INVALID_PROCESS_ID;

      /* Notification the client */

      DEBUGVERIFY(nxsig_notification(pid, &event,
                                     SI_MESGQ, &msgq->ntwork));
    }
#endif

  /* Check if any tasks are waiting for the MQ not empty event. */

  if (msgq->cmn.nwaitnotempty > 0)
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Find the highest priority task that is waiting for
       * this queue to be non-empty in waitfornotempty
       * list. leave_critical_section() should give us sufficient
       * protection since interrupts should never cause a change
       * in this list
       */

      btcb = (FAR struct tcb_s *)dq_remfirst(MQ_WNELIST(msgq->cmn));

      /* If one was found, unblock it */

      DEBUGASSERT(btcb);

      if (WDOG_ISACTIVE(&btcb->waitdog))
        {
          wd_cancel(&btcb->waitdog);
        }

      msgq->cmn.nwaitnotempty--;

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
