/****************************************************************************
 *  sched/mqueue/mq_rcvinternal.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_verify_receive
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_receive and
 *   [nx]mq_timedreceive.  This function verifies the input parameters that
 *   are common to both functions.
 *
 * Input Parameters:
 *   mqdes - Message Queue Descriptor
 *   msg - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *
 * Returned Value:
 *   One success, zero (OK) is returned.  A negated errno value is returned
 *   on any failure:
 *
 *   EPERM    Message queue opened not opened for reading.
 *   EMSGSIZE 'msglen' was less than the maxmsgsize attribute of the message
 *            queue.
 *   EINVAL   Invalid 'msg' or 'mqdes'
 *
 ****************************************************************************/

int nxmq_verify_receive(mqd_t mqdes, FAR char *msg, size_t msglen)
{
  /* Verify the input parameters */

  if (!msg || !mqdes)
    {
      return -EINVAL;
    }

  if ((mqdes->oflags & O_RDOK) == 0)
    {
      return -EPERM;
    }

  if (msglen < (size_t)mqdes->msgq->maxmsgsize)
    {
      return -EMSGSIZE;
    }

  return OK;
}

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
 *   mqdes  - Message queue descriptor
 *   rcvmsg - The caller-provided location in which to return the newly
 *            received message.
 *
 * Returned Value:
 *   One success, zero (OK) is returned.  A negated errno value is returned
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

int nxmq_wait_receive(mqd_t mqdes, FAR struct mqueue_msg_s **rcvmsg)
{
  FAR struct tcb_s *rtcb;
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *newmsg;
  int ret;

  DEBUGASSERT(rcvmsg != NULL);
  *rcvmsg = NULL;  /* Assume failure */

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

  /* Get a pointer to the message queue */

  msgq = mqdes->msgq;

  /* Get the message from the head of the queue */

  while ((newmsg = (FAR struct mqueue_msg_s *)sq_remfirst(&msgq->msglist))
          == NULL)
    {
      /* The queue is empty!  Should we block until there the above condition
       * has been satisfied?
       */

      if ((mqdes->oflags & O_NONBLOCK) == 0)
        {
          /* Yes.. Block and try again */

          rtcb           = this_task();
          rtcb->msgwaitq = msgq;
          msgq->nwaitnotempty++;

          /* Initialize the 'errcode" used to communication wake-up error
           * conditions.
           */

          rtcb->errcode  = OK;

          /* Make sure this is not the idle task, descheduling that
           * isn't going to end well.
           */

          DEBUGASSERT(NULL != rtcb->flink);
          up_block_task(rtcb, TSTATE_WAIT_MQNOTEMPTY);

          /* When we resume at this point, either (1) the message queue
           * is no longer empty, or (2) the wait has been interrupted by
           * a signal.  We can detect the latter case be examining the
           * errno value (should be either EINTR or ETIMEDOUT).
           */

          ret = rtcb->errcode;
          if (ret != OK)
            {
              return -ret;
            }
        }
      else
        {
          /* The queue was empty, and the O_NONBLOCK flag was set for the
           * message queue description referred to by 'mqdes'.
           */

          return -EAGAIN;
        }
    }

  /* If we got message, then decrement the number of messages in
   * the queue while we are still in the critical section
   */

  if (newmsg)
    {
      msgq->nmsgs--;
    }

  *rcvmsg = newmsg;
  return OK;
}

/****************************************************************************
 * Name: nxmq_do_receive
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_receive and
 *   [nx]mq_timedreceive.  This function accepts the message obtained by
 *   mq_waitmsg, provides the message content to the user, notifies any
 *   threads that were waiting for the message queue to become non-full,
 *   and disposes of the message structure
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor
 *   mqmsg   - The message obtained by mq_waitmsg()
 *   ubuffer - The address of the user provided buffer to receive the message
 *   prio    - The user-provided location to return the message priority.
 *
 * Returned Value:
 *   Returns the length of the received message.  This function does not
 *   fail.
 *
 * Assumptions:
 * - The caller has provided all validity checking of the input parameters
 *   using nxmq_verify_receive.
 * - The user buffer, ubuffer, is known to be large enough to accept the
 *   largest message that an be sent on this message queue
 * - Pre-emption should be disabled throughout this call.
 *
 ****************************************************************************/

ssize_t nxmq_do_receive(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
                        FAR char *ubuffer, unsigned int *prio)
{
  FAR struct tcb_s *btcb;
  irqstate_t flags;
  FAR struct mqueue_inode_s *msgq;
  ssize_t rcvmsglen;

  /* Get the length of the message (also the return value) */

  rcvmsglen = mqmsg->msglen;

  /* Copy the message into the caller's buffer */

  memcpy(ubuffer, (FAR const void *)mqmsg->mail, rcvmsglen);

  /* Copy the message priority as well (if a buffer is provided) */

  if (prio)
    {
      *prio = mqmsg->priority;
    }

  /* We are done with the message.  Deallocate it now. */

  nxmq_free_msg(mqmsg);

  /* Check if any tasks are waiting for the MQ not full event. */

  msgq = mqdes->msgq;
  if (msgq->nwaitnotfull > 0)
    {
      /* Find the highest priority task that is waiting for
       * this queue to be not-full in g_waitingformqnotfull list.
       * This must be performed in a critical section because
       * messages can be sent from interrupt handlers.
       */

      flags = enter_critical_section();
      for (btcb = (FAR struct tcb_s *)g_waitingformqnotfull.head;
           btcb && btcb->msgwaitq != msgq;
           btcb = btcb->flink)
        {
        }

      /* If one was found, unblock it.  NOTE:  There is a race
       * condition here:  the queue might be full again by the
       * time the task is unblocked
       */

      DEBUGASSERT(btcb != NULL);

      btcb->msgwaitq = NULL;
      msgq->nwaitnotfull--;
      up_unblock_task(btcb);

      leave_critical_section(flags);
    }

  /* Return the length of the message transferred to the user buffer */

  return rcvmsglen;
}
