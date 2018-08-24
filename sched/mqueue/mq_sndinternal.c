/****************************************************************************
 *  sched/mqueue/mq_sndinternal.c
 *
 *   Copyright (C) 2007, 2009, 2013-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#ifndef CONFIG_DISABLE_SIGNALS
#  include "signal/signal.h"
#endif
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
 *   mqdes - Message queue descriptor
 *   msg - Message to send
 *   msglen - The length of the message in bytes
 *   prio - The priority of the message
 *
 * Returned Value:
 *   One success, 0 (OK) is returned. On failure, a negated errno value is
 *   returned.
 *
 *     EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *     EPERM    Message queue opened not opened for writing.
 *     EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *               message queue.
 *
 ****************************************************************************/

int nxmq_verify_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
                     int prio)
{
  /* Verify the input parameters */

  if (!msg || !mqdes || prio < 0 || prio > MQ_PRIO_MAX)
    {
      return -EINVAL;
    }

  if ((mqdes->oflags & O_WROK) == 0)
    {
      return -EPERM;
    }

  if (msglen > (size_t)mqdes->msgq->maxmsgsize)
    {
      return -EMSGSIZE;
    }

  return OK;
}

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
  irqstate_t flags;

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

      flags = enter_critical_section();
      mqmsg = (FAR struct mqueue_msg_s *)sq_remfirst(&g_msgfree);
      leave_critical_section(flags);

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
              /* Yes... remember that this message was dynamically allocated */

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
 *   mqdes - Message queue descriptor
 *
 * Returned Value:
 *   On success, nxmq_wait_send() returns 0 (OK); a negated errno value is
 *   returned on any failure:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINTR    The call was interrupted by a signal handler.
 *   ETIMEOUT A timeout expired before the message queue became non-full
 *            (mq_timedsend only).
 *
 * Assumptions/restrictions:
 * - The caller has verified the input parameters using nxmq_verify_send().
 * - Executes within a critical section established by the caller.
 *
 ****************************************************************************/

int nxmq_wait_send(mqd_t mqdes)
{
  FAR struct tcb_s *rtcb;
  FAR struct mqueue_inode_s *msgq;
  int ret;

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

  /* Get a pointer to the message queue */

  msgq = mqdes->msgq;

  /* Verify that the queue is indeed full as the caller thinks */

  if (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Should we block until there is sufficient space in the
       * message queue?
       */

      if ((mqdes->oflags & O_NONBLOCK) != 0)
        {
          /* No... We will return an error to the caller. */

          return -EAGAIN;
        }

      /* Yes... We will not return control until the message queue is
       * available or we receive a signal or at timout occurs.
       */

      else
        {
          /* Loop until there are fewer than max allowable messages in the
           * receiving message queue
           */

          while (msgq->nmsgs >= msgq->maxmsgs)
            {
              int saved_errno;

              /* Block until the message queue is no longer full.
               * When we are unblocked, we will try again
               */

              rtcb           = this_task();
              rtcb->msgwaitq = msgq;
              msgq->nwaitnotfull++;

              /* "Borrow" the per-task errno to communication wake-up error
               * conditions.
               */

              saved_errno   = rtcb->pterrno;
              rtcb->pterrno = OK;

              up_block_task(rtcb, TSTATE_WAIT_MQNOTFULL);

              /* When we resume at this point, either (1) the message queue
               * is no longer empty, or (2) the wait has been interrupted by
               * a signal.  We can detect the latter case be examining the
               * per-task errno value (should be EINTR or ETIMEOUT).
               */

              ret           = rtcb->pterrno;
              rtcb->pterrno = saved_errno;

              if (ret != OK)
                {
                  return -ret;
                }
            }
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
 *   message queue (mqdes).  Then it notifies any tasks that were waiting
 *   for message queue notifications setup by mq_notify.  And, finally, it
 *   awakens any tasks that were waiting for the message not empty event.
 *
 * Input Parameters:
 *   mqdes  - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   This function always returns OK.
 *
 ****************************************************************************/

int nxmq_do_send(mqd_t mqdes, FAR struct mqueue_msg_s *mqmsg,
                 FAR const char *msg, size_t msglen, int prio)
{
  FAR struct tcb_s *btcb;
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *next;
  FAR struct mqueue_msg_s *prev;
  irqstate_t flags;

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* Construct the message header info */

  mqmsg->priority = prio;
  mqmsg->msglen   = msglen;

  /* Copy the message data into the message */

  memcpy((FAR void *)mqmsg->mail, (FAR const void *)msg, msglen);

  /* Insert the new message in the message queue */

  flags = enter_critical_section();

  /* Search the message list to find the location to insert the new
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

  msgq->nmsgs++;
  leave_critical_section(flags);

  /* Check if we need to notify any tasks that are attached to the
   * message queue
   */

#ifndef CONFIG_DISABLE_SIGNALS
  if (msgq->ntmqdes)
    {
      struct sigevent event;
      pid_t pid;

      /* Remove the message notification data from the message queue. */

      memcpy(&event, &msgq->ntevent, sizeof(struct sigevent));
      pid = msgq->ntpid;

      /* Detach the notification */

      memset(&msgq->ntevent, 0, sizeof(struct sigevent));
      msgq->ntpid   = INVALID_PROCESS_ID;
      msgq->ntmqdes = NULL;

      /* Notification the client via signal? */

      if (event.sigev_notify == SIGEV_SIGNAL)
        {
          /* Yes... Queue the signal -- What if this returns an error? */

#ifdef CONFIG_CAN_PASS_STRUCTS
          DEBUGVERIFY(nxsig_mqnotempty(pid, event.sigev_signo,
                                       event.sigev_value));
#else
          DEBUGVERIFY(nxsig_mqnotempty(pid, event.sigev_signo,
                                       event.sigev_value.sival_ptr));
#endif
        }

#ifdef CONFIG_SIG_EVTHREAD
      /* Notify the client via a function call */

      else if (event.sigev_notify == SIGEV_THREAD)
        {
          DEBUGVERIFY(nxsig_notification(pid, &event));
        }
#endif

    }
#endif

  /* Check if any tasks are waiting for the MQ not empty event. */

  flags = enter_critical_section();
  if (msgq->nwaitnotempty > 0)
    {
      /* Find the highest priority task that is waiting for
       * this queue to be non-empty in g_waitingformqnotempty
       * list. sched_lock() should give us sufficent protection since
       * interrupts should never cause a change in this list
       */

      for (btcb = (FAR struct tcb_s *)g_waitingformqnotempty.head;
           btcb && btcb->msgwaitq != msgq;
           btcb = btcb->flink);

      /* If one was found, unblock it */

      DEBUGASSERT(btcb);

      btcb->msgwaitq = NULL;
      msgq->nwaitnotempty--;
      up_unblock_task(btcb);
    }

  leave_critical_section(flags);
  sched_unlock();
  return OK;
}
