/****************************************************************************
 * sched/mqueue/mq_sndinternal.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_verify_send
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_send and
 *   [nx]mq_timesend.  This function verifies the input parameters that are
 *   common to both functions.
 *
 * Input Parameters:
 *   msgq   - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   On success, 0 (OK) is returned. On failure, a negated errno value is
 *   returned.
 *
 *     EINVAL   Either msg or msgq is NULL or the value of prio is invalid.
 *     EPERM    Message queue opened not opened for writing.
 *     EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *               message queue.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int nxmq_verify_send(FAR FAR struct file *mq, FAR const char *msg,
                     size_t msglen, unsigned int prio)
{
  FAR struct inode *inode = mq->f_inode;
  FAR struct mqueue_inode_s *msgq;

  if (inode == NULL)
    {
      return -EBADF;
    }

  msgq = inode->i_private;

  /* Verify the input parameters */

  if (msg == NULL || msgq == NULL || prio > MQ_PRIO_MAX)
    {
      return -EINVAL;
    }

  if ((mq->f_oflags & O_WROK) == 0)
    {
      return -EPERM;
    }

  if (msglen > (size_t)msgq->maxmsgsize)
    {
      return -EMSGSIZE;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxmq_alloc_msg
 *
 * Description:
 *   The nxmq_alloc_msg function will get a free message for use by the
 *   operating system.  The message will be allocated from the g_msgfree
 *   list.
 *
 *   If the list is empty AND the message is NOT being allocated from the
 *   interrupt level, then the message will be allocated.  If a message
 *   cannot be obtained, the operating system is dead and therefore cannot
 *   continue.
 *
 *   If the list is empty AND the message IS being allocated from the
 *   interrupt level.  This function will attempt to get a message from
 *   the g_msgfreeirq list.  If this is unsuccessful, the calling interrupt
 *   handler will be notified.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the allocated msg structure.  On a failure to allocate,
 *   this function PANICs.
 *
 ****************************************************************************/

FAR struct mqueue_msg_s *nxmq_alloc_msg(void)
{
  FAR struct mqueue_msg_s *mqmsg;

  /* If we were called from an interrupt handler, then try to get the message
   * from generally available list of messages. If this fails, then try the
   * list of messages reserved for interrupt handlers
   */

  if (up_interrupt_context())
    {
      /* Try the general free list */

      mqmsg = (FAR struct mqueue_msg_s *)sq_remfirst(&g_msgfree);
      if (mqmsg == NULL)
        {
          /* Try the free list reserved for interrupt handlers */

          mqmsg = (FAR struct mqueue_msg_s *)sq_remfirst(&g_msgfreeirq);
        }
    }

  /* We were not called from an interrupt handler. */

  else
    {
      /* Try to get the message from the generally available free list.
       * Disable interrupts -- we might be called from an interrupt handler.
       */

      mqmsg = (FAR struct mqueue_msg_s *)sq_remfirst(&g_msgfree);

      /* If we cannot a message from the free list, then we will have to
       * allocate one.
       */

      if (mqmsg == NULL)
        {
          mqmsg = (FAR struct mqueue_msg_s *)
            kmm_malloc((sizeof (struct mqueue_msg_s)));

          /* Check if we allocated the message */

          if (mqmsg != NULL)
            {
              /* Yes... remember that this message was dynamically
               * allocated.
               */

              mqmsg->type = MQ_ALLOC_DYN;
            }
        }
    }

  return mqmsg;
}

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
 *   oflags - flags from user set
 *
 * Returned Value:
 *   On success, nxmq_wait_send() returns 0 (OK); a negated errno value is
 *   returned on any failure:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by msgq.
 *   EINTR    The call was interrupted by a signal handler.
 *   ETIMEOUT A timeout expired before the message queue became non-full
 *            (mq_timedsend only).
 *
 * Assumptions/restrictions:
 * - The caller has verified the input parameters using nxmq_verify_send().
 * - Executes within a critical section established by the caller.
 *
 ****************************************************************************/

int nxmq_wait_send(FAR struct mqueue_inode_s *msgq, int oflags)
{
  FAR struct tcb_s *rtcb;

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

  /* Verify that the queue is indeed full as the caller thinks */

  /* Loop until there are fewer than max allowable messages in the
   * receiving message queue
   */

  while (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Should we block until there is sufficient space in the
       * message queue?
       */

      if ((oflags & O_NONBLOCK) != 0)
        {
          /* No... We will return an error to the caller. */

          return -EAGAIN;
        }

      /* Block until the message queue is no longer full.
       * When we are unblocked, we will try again
       */

      rtcb           = this_task();
      rtcb->msgwaitq = msgq;
      msgq->nwaitnotfull++;

      /* Initialize the errcode used to communication wake-up error
       * conditions.
       */

      rtcb->errcode = OK;

      /* Make sure this is not the idle task, descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(NULL != rtcb->flink);
      up_block_task(rtcb, TSTATE_WAIT_MQNOTFULL);

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
 * Name: nxmq_do_send
 *
 * Description:
 *   This is internal, common logic shared by both [nx]mq_send and
 *   [nx]mq_timesend.  This function adds the specified message (msg) to the
 *   message queue (msgq).  Then it notifies any tasks that were waiting
 *   for message queue notifications setup by mq_notify.  And, finally, it
 *   awakens any tasks that were waiting for the message not empty event.
 *
 * Input Parameters:
 *   msgq   - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   This function always returns OK.
 *
 ****************************************************************************/

int nxmq_do_send(FAR struct mqueue_inode_s *msgq,
                 FAR struct mqueue_msg_s *mqmsg,
                 FAR const char *msg, size_t msglen, unsigned int prio)
{
  FAR struct tcb_s *btcb;
  FAR struct mqueue_msg_s *next;
  FAR struct mqueue_msg_s *prev;

  /* Construct the message header info */

  mqmsg->priority = prio;
  mqmsg->msglen   = msglen;

  /* Copy the message data into the message */

  memcpy((FAR void *)mqmsg->mail, (FAR const void *)msg, msglen);

  /* Insert the new message in the message queue
   * Search the message list to find the location to insert the new
   * message. Each is list is maintained in ascending priority order.
   */

  for (prev = NULL, next = (FAR struct mqueue_msg_s *)msgq->msglist.head;
       next && prio <= next->priority;
       prev = next, next = next->next);

  /* Add the message at the right place */

  if (prev)
    {
      sq_addafter((FAR sq_entry_t *)prev, (FAR sq_entry_t *)mqmsg,
                  &msgq->msglist);
    }
  else
    {
      sq_addfirst((FAR sq_entry_t *)mqmsg, &msgq->msglist);
    }

  /* Increment the count of messages in the queue */

  if (msgq->nmsgs++ == 0)
    {
      nxmq_pollnotify(msgq, POLLIN);
    }

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

  if (msgq->nwaitnotempty > 0)
    {
      /* Find the highest priority task that is waiting for
       * this queue to be non-empty in g_waitingformqnotempty
       * list. leave_critical_section() should give us sufficient
       * protection since interrupts should never cause a change
       * in this list
       */

      for (btcb = (FAR struct tcb_s *)g_waitingformqnotempty.head;
           btcb && btcb->msgwaitq != msgq;
           btcb = btcb->flink)
        {
        }

      /* If one was found, unblock it */

      DEBUGASSERT(btcb);

      btcb->msgwaitq = NULL;
      msgq->nwaitnotempty--;
      up_unblock_task(btcb);
    }

  return OK;
}
