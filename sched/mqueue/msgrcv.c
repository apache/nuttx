/****************************************************************************
 * sched/mqueue/msgrcv.c
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
 * Name: msgrcv_wait
 ****************************************************************************/

static int msgrcv_wait(FAR struct msgq_s *msgq, FAR struct msgbuf_s **rcvmsg,
                       long msgtyp, int msgflg)
{
  FAR struct msgbuf_s *newmsg = NULL;
  FAR struct msgbuf_s *tmp;
  FAR struct tcb_s *rtcb;
  bool switch_needed;

#ifdef CONFIG_CANCELLATION_POINTS
  /* msgrcv_wait() is not a cancellation point, but it may be called
   * from msgrcv() which are cancellation point.
   */

  if (check_cancellation_point())
    {
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      return -ECANCELED;
    }
#endif

  /* Get the message from the head of the queue */

  while (1)
    {
      list_for_every_entry(&msgq->msglist, tmp, struct msgbuf_s, node)
        {
          /* Unless MSG_COPY is specified in msgflg (see below), the msgtyp
           * argument specifies the type of message requested, as follows:
           *
           * 1. If msgtyp is 0, then the first message in the queue is read.
           *
           * 2. If msgtyp is greater than 0, then the first message in the
           *    queue of type msgtyp is read, unless MSG_EXCEPT was
           *    specified in msgflg, in which case the first message in the
           *    queue of type not equal to msgtyp will be read.
           *
           * 3. If msgtyp is less than 0, then the first message in the
           *    queue with the lowest type less than or equal to the
           *    absolute value of msgtyp will be read.
           */

          if (msgtyp < 0)
            {
              if (newmsg == NULL || newmsg->mtype > tmp->mtype)
                {
                  newmsg = tmp;
                }
            }
          else if (msgtyp == 0 ||
                   ((msgflg & MSG_EXCEPT) != 0) == (tmp->mtype != msgtyp))
            {
              newmsg = tmp;
              break;
            }
        }

      if (newmsg)
        {
          list_delete(&newmsg->node);
          goto found;
        }

      if ((msgflg & IPC_NOWAIT) != 0)
        {
          return -EAGAIN;
        }

      /* The queue is empty!  Should we block until there the above condition
       * has been satisfied?
       */

      rtcb          = this_task();
      rtcb->waitobj = msgq;
      msgq->cmn.nwaitnotempty++;

      /* Initialize the 'errcode" used to communication wake-up error
       * conditions.
       */

      rtcb->errcode = OK;

      /* Make sure this is not the idle task, descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(NULL != rtcb->flink);

      /* Remove the tcb task from the ready-to-run list. */

      switch_needed = nxsched_remove_readytorun(rtcb, true);

      /* Add the task to the specified blocked task list */

      rtcb->task_state = TSTATE_WAIT_MQNOTEMPTY;
      nxsched_add_prioritized(rtcb, MQ_WNELIST(msgq->cmn));

      /* Now, perform the context switch if one is needed */

      if (switch_needed)
        {
          up_block_task(rtcb);
        }

      /* When we resume at this point, either (1) the message queue
       * is no longer empty, or (2) the wait has been interrupted by
       * a signal.  We can detect the latter case be examining the
       * errno value (should be either EINTR or ETIMEDOUT).
       */

      if (rtcb->errcode != OK)
        {
          return -rtcb->errcode;
        }
    }

found:
  msgq->nmsgs--;
  *rcvmsg = newmsg;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: msgrcv
 *
 * Description:
 *   The msgrcv() function reads a message from the message queue specified
 *   by the msqid parameter and places it in the user-defined buffer
 *   pointed to by the *msgp parameter.
 *
 * Input Parameters:
 *   msqid  - Message queue identifier
 *   msgp   - Pointer to a buffer in which the received message will be
 *            stored
 *   msgsz  - Length of the data part of the buffer
 *   msgtyp - Type of message to be received.
 *   msgflg - Operations flags.
 *
 * Returned Value:
 *   On success, msgrcv() returns the number of bytes actually copied
 *   into the mtext array.
 *   On failure, both functions return -1, and set errno to indicate
 *   the error.
 *
 ****************************************************************************/

ssize_t msgrcv(int msqid, FAR void *msgp, size_t msgsz, long msgtyp,
               int msgflg)
{
  FAR struct msgbuf_s *msg = NULL;
  FAR struct mymsg *buf = msgp;
  FAR struct msgq_s *msgq;
  FAR struct tcb_s *btcb;
  irqstate_t flags;
  int ret;

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

  if (msgsz < msgq->maxmsgsize &&
      ((msgflg & MSG_NOERROR) == 0))
    {
      ret = -EMSGSIZE;
      goto errout_with_critical;
    }

  ret = msgrcv_wait(msgq, &msg, msgtyp, msgflg);
  if (ret < 0)
    {
      goto errout_with_critical;
    }

  ret = msgsz > msg->msize ? msg->msize : msgsz;
  buf->mtype = msg->mtype;
  memcpy(buf->mtext, msg->mtext, ret);

  list_add_tail(&g_msgfreelist, &msg->node);

  /* Check if any tasks are waiting for the MQ not full event. */

  if (msgq->cmn.nwaitnotfull > 0)
    {
      FAR struct tcb_s *rtcb = this_task();

      /* Find the highest priority task that is waiting for
       * this queue to be not-full in g_waitingformqnotfull list.
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
          up_unblock_task(btcb, rtcb);
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

  return ret;
}
