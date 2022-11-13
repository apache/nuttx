/****************************************************************************
 * sched/mqueue/msgsnd.c
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

#include <assert.h>

#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "mqueue/msg.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msgsnd_wait
 ****************************************************************************/

static int msgsnd_wait(FAR struct msgq_s *msgq, int msgflg)
{
  FAR struct tcb_s *rtcb;
  bool switch_needed;

#ifdef CONFIG_CANCELLATION_POINTS
  /* msgsnd_wait() is not a cancellation point, but may be called via
   * msgsnd() which are cancellation points.
   */

  if (check_cancellation_point())
    {
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      return -ECANCELED;
    }
#endif

  /* Verify that the queue is indeed full as the caller thinks
   * Loop until there are fewer than max allowable messages in the
   * receiving message queue
   */

  while (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Should we block until there is sufficient space in the
       * message queue?
       */

      if ((msgflg & IPC_NOWAIT) != 0)
        {
          return -EAGAIN;
        }

      /* Block until the message queue is no longer full.
       * When we are unblocked, we will try again
       */

      rtcb          = this_task();
      rtcb->waitobj = msgq;
      msgq->cmn.nwaitnotfull++;

      /* Initialize the errcode used to communication wake-up error
       * conditions.
       */

      rtcb->errcode  = OK;

      /* Make sure this is not the idle task, descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(NULL != rtcb->flink);

      /* Remove the tcb task from the ready-to-run list. */

      switch_needed = nxsched_remove_readytorun(rtcb, true);

      /* Add the task to the specified blocked task list */

      rtcb->task_state = TSTATE_WAIT_MQNOTFULL;
      nxsched_add_prioritized(rtcb, MQ_WNFLIST(msgq->cmn));

      /* Now, perform the context switch if one is needed */

      if (switch_needed)
        {
          up_block_task(rtcb);
        }

      /* When we resume at this point, either (1) the message queue
       * is no longer empty, or (2) the wait has been interrupted by
       * a signal.  We can detect the latter case be examining the
       * per-task errno value (should be EINTR or ETIMEOUT).
       */

      if (rtcb->errcode != OK)
        {
          return -rtcb->errcode;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msgsnd
 *
 * Description:
 *   The msgsnd() function is used to send a message to the queue
 *   associated with the message queue identifier specified by msqid.
 *   The msgp argument points to a user-defined buffer that must contain
 *   first a field of type long int that will specify the type of the
 *   message, and then a data portion that will hold the data bytes of
 *   the message.
 *
 * Input Parameters:
 *   msqid  - Message queue identifier
 *   msgp   - Pointer to a buffer with the message to be sent
 *   msgsz  - Length of the data part of the message to be sent
 *   msgflg - Operations flags
 *
 * Returned Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int msgsnd(int msqid, FAR const void *msgp, size_t msgsz, int msgflg)
{
  FAR const struct mymsg *buf = msgp;
  FAR struct msgbuf_s *msg;
  FAR struct msgq_s *msgq;
  FAR struct tcb_s *btcb;
  irqstate_t flags;
  int ret = OK;

  if (msgp == NULL)
    {
      ret = -EFAULT;
      goto errout;
    }

  flags = enter_critical_section();

  msgq = nxmsg_lookup(msqid);
  if (msgq == NULL)
    {
      ret = -EINVAL;
      goto errout_with_critical;
    }

  if (msgsz > msgq->maxmsgsize)
    {
      ret = -EMSGSIZE;
      goto errout_with_critical;
    }

  /* Is the message queue FULL? */

  if (msgq->nmsgs >= msgq->maxmsgs)
    {
      if (!up_interrupt_context()) /* In an interrupt handler? */
        {
          /* Yes.. the message queue is full.  Wait for space to become
           * available in the message queue.
           */

          ret = msgsnd_wait(msgq, msgflg);
        }
      else if ((msgflg & IPC_NOWAIT) != 0)
        {
          ret = -EAGAIN;
        }
    }

  if (ret == OK)
    {
      /* Now allocate the message. */

      msg = (FAR struct msgbuf_s *)list_remove_head(&g_msgfreelist);
      if (msg == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_critical;
        }

      /* Check if the message was successfully allocated */

      msg->msize = msgsz;
      msg->mtype = buf->mtype;
      memcpy(msg->mtext, buf->mtext, msgsz);

      /* Insert the new message in the message queue */

      list_add_tail(&msgq->msglist, &msg->node);

      msgq->nmsgs++;

      if (msgq->cmn.nwaitnotempty > 0)
        {
          FAR struct tcb_s *rtcb = this_task();

          /* Find the highest priority task that is waiting for
           * this queue to be non-empty in g_waitingformqnotempty
           * list. enter_critical_section() should give us sufficient
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
              up_unblock_task(btcb, rtcb);
            }
        }
    }

errout_with_critical:
  leave_critical_section(flags);
errout:
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
