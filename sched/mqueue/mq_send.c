/****************************************************************************
 * sched/mqueue/mq_send.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <mqueue.h>
#include <sys/types.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/cancelpt.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/irq.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
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
 *     EBADF    Message queue opened not opened for writing.
 *     EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *               message queue.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int nxmq_verify_send(FAR FAR struct file *mq, FAR const char *msg,
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

  if (msg == NULL || msgq == NULL || prio >= MQ_PRIO_MAX)
    {
      return -EINVAL;
    }

  if ((mq->f_oflags & O_WROK) == 0)
    {
      return -EBADF;
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

static FAR struct mqueue_msg_s *nxmq_alloc_msg(uint16_t msgsize)
{
  FAR struct mqueue_msg_s *mqmsg;
  irqstate_t flags;

  /* Try to get the message from the generally available free list. */

  flags = spin_lock_irqsave(&g_msgfreelock);
  mqmsg = (FAR struct mqueue_msg_s *)list_remove_head(&g_msgfree);
  spin_unlock_irqrestore(&g_msgfreelock, flags);
  if (mqmsg == NULL)
    {
      /* If we were called from an interrupt handler, then try to get the
       * message from generally available list of messages. If this fails,
       * then try the list of messages reserved for interrupt handlers
       */

      if (up_interrupt_context())
        {
          /* Try the free list reserved for interrupt handlers */

          flags = spin_lock_irqsave(&g_msgfreelock);
          mqmsg = (FAR struct mqueue_msg_s *)list_remove_head(&g_msgfreeirq);
          spin_unlock_irqrestore(&g_msgfreelock, flags);
        }

      /* We were not called from an interrupt handler. */

      else
        {
          /* If we cannot a message from the free list, then we will have to
           * allocate one.
           */

          mqmsg = kmm_malloc(MQ_MSG_SIZE(msgsize));

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
 * Name: nxmq_add_queue
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
 *
 * Returned Value:
 *   This function always returns OK.
 *
 ****************************************************************************/

static void nxmq_add_queue(FAR struct mqueue_inode_s *msgq,
                           FAR struct mqueue_msg_s *mqmsg,
                           unsigned int prio)
{
  FAR struct mqueue_msg_s *prev = NULL;
  FAR struct mqueue_msg_s *next;

  /* Insert the new message in the message queue
   * Search the message list to find the location to insert the new
   * message. Each is list is maintained in ascending priority order.
   */

  list_for_every_entry(&msgq->msglist, next, struct mqueue_msg_s, node)
    {
      if (prio > next->priority)
        {
          break;
        }
      else
        {
          prev = next;
        }
    }

  /* Add the message at the right place */

  if (prev)
    {
      list_add_after(&prev->node, &mqmsg->node);
    }
  else
    {
      list_add_head(&msgq->msglist, &mqmsg->node);
    }
}

/****************************************************************************
 * Name: file_mq_timedsend_internal
 *
 * Description:
 *   This is an internal function of file_mq_timedsend()/file_mq_ticksend(),
 *   please refer to the detailed description for more information.
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *   ticks   - Ticks to wait from the start time until the semaphore is
 *             posted.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedsend() for the list list valid return values).
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mq.
 *   EINVAL   Either msg or mq is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

static
int file_mq_timedsend_internal(FAR struct file *mq, FAR const char *msg,
                               size_t msglen, unsigned int prio,
                               FAR const struct timespec *abstime,
                               sclock_t ticks)
{
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *mqmsg;
  irqstate_t flags;
  int ret = 0;

  /* Verify the input parameters */

  if (abstime && (abstime->tv_nsec < 0 || abstime->tv_nsec >= 1000000000))
    {
      return -EINVAL;
    }

  if (mq == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_DEBUG_FEATURES
  /* Verify the input parameters on any failures to verify. */

  ret = nxmq_verify_send(mq, msg, msglen, prio);
  if (ret < 0)
    {
      return ret;
    }
#endif

  msgq = mq->f_inode->i_private;

  /* Pre-allocate a message structure */

  mqmsg = nxmq_alloc_msg(msglen);
  if (!mqmsg)
    {
      return -ENOMEM;
    }

  memcpy(mqmsg->mail, msg, msglen);
  mqmsg->priority = prio;
  mqmsg->msglen   = msglen;

  /* Disable interruption */

  flags = enter_critical_section();

  if (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Verify that the message is full and we can't wait */

      if ((up_interrupt_context() || (mq->f_oflags & O_NONBLOCK) != 0))
        {
          ret = -EAGAIN;
          goto out;
        }

      /* The message queue is full.  We will need to wait for the message
       * queue to become non-full.
       */

      ret = nxmq_wait_send(msgq, abstime, ticks);
      if (ret < 0)
        {
          goto out;
        }
    }

  /* Add the message to the message queue */

  nxmq_add_queue(msgq, mqmsg, prio);

  /* Increment the count of messages in the queue */

  if (msgq->nmsgs++ == 0)
    {
      nxmq_pollnotify(msgq, POLLIN);
    }

  /* Notify any tasks that are waiting for a message to become available */

  nxmq_notify_send(msgq);

out:
  leave_critical_section(flags);

  if (ret < 0)
    {
      nxmq_free_msg(mqmsg);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mq_timedsend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mq).  file_mq_timedsend() behaves just like mq_send(), except that if
 *   the queue is full and the O_NONBLOCK flag is not enabled for the
 *   message queue description, then abstime points to a structure which
 *   specifies a ceiling on the time for which the call will block.
 *
 *   file_mq_timedsend() is functionally equivalent to mq_timedsend() except
 *   that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedsend() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedsend() for the list list valid return values).
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mq.
 *   EINVAL   Either msg or mq is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int file_mq_timedsend(FAR struct file *mq, FAR const char *msg,
                      size_t msglen, unsigned int prio,
                      FAR const struct timespec *abstime)
{
  return file_mq_timedsend_internal(mq, msg, msglen, prio, abstime, -1);
}

/****************************************************************************
 * Name: file_mq_ticksend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mq).  file_mq_ticksend() behaves just like mq_send(), except that if
 *   the queue is full and the O_NONBLOCK flag is not enabled for the
 *   message queue description, then abstime points to a structure which
 *   specifies a ceiling on the time for which the call will block.
 *
 *   file_mq_ticksend() is functionally equivalent to mq_timedsend() except
 *   that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedsend() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   ticks   - Ticks to wait from the start time until the semaphore is
 *             posted.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedsend() for the list list valid return values).
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mq.
 *   EINVAL   Either msg or mq is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int file_mq_ticksend(FAR struct file *mq, FAR const char *msg,
                     size_t msglen, unsigned int prio, sclock_t ticks)
{
  return file_mq_timedsend_internal(mq, msg, msglen, prio, NULL, ticks);
}

/****************************************************************************
 * Name: nxmq_timedsend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  nxmq_timedsend() behaves just like mq_send(), except
 *   that if the queue is full and the O_NONBLOCK flag is not enabled for
 *   the message queue description, then abstime points to a structure which
 *   specifies a ceiling on the time for which the call will block.
 *
 *   nxmq_timedsend() is functionally equivalent to mq_timedsend() except
 *   that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_timedsend() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes   - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_timedsend() for the list list valid return values).
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int nxmq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen,
                   unsigned int prio, FAR const struct timespec *abstime)
{
  FAR struct file *filep;
  int ret;

  ret = fs_getfilep(mqdes, &filep);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_mq_timedsend_internal(filep, msg, msglen, prio, abstime, -1);
  fs_putfilep(filep);
  return ret;
}

/****************************************************************************
 * Name: mq_timedsend
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  The "msglen" parameter specifies the length of the message
 *   in bytes pointed to by "msg."  This length must not exceed the maximum
 *   message length from the mq_getattr().
 *
 *   If the message queue is not full, mq_timedsend() place the message in
 *   the message queue at the position indicated by the "prio" argrument.
 *   Messages with higher priority will be inserted before lower priority
 *   messages.  The value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK is not set in the
 *   message queue, then mq_timedsend() will block until space becomes
 *   available to the queue the message or a timeout occurs.
 *
 *   mq_timedsend() behaves just like mq_send(), except that if the queue
 *   is full and the O_NONBLOCK flag is not enabled for the message queue
 *   description, then abstime points to a structure which specifies a
 *   ceiling on the time for which the call will block.  This ceiling is an
 *   absolute timeout in seconds and nanoseconds since the Epoch (midnight
 *   on the morning of 1 January 1970).
 *
 *   If the message queue is full, and the timeout has already expired by
 *   the time of the call, mq_timedsend() returns immediately.
 *
 * Input Parameters:
 *   mqdes   - Message queue descriptor
 *   msg     - Message to send
 *   msglen  - The length of the message in bytes
 *   prio    - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *
 * Returned Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was full, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ****************************************************************************/

int mq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen,
                 unsigned int prio, FAR const struct timespec *abstime)
{
  int ret;

  /* mq_timedsend() is a cancellation point */

  enter_cancellation_point();

  /* Let nxmq_send() do all of the work */

  ret = nxmq_timedsend(mqdes, msg, msglen, prio, abstime);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}

/****************************************************************************
 * Name: file_mq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mq).  This is an internal OS interface.  It is functionally
 *   equivalent to mq_send() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_send() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq     - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_send() for the list list valid return values).
 *
 ****************************************************************************/

int file_mq_send(FAR struct file *mq, FAR const char *msg, size_t msglen,
                 unsigned int prio)
{
  return file_mq_timedsend_internal(mq, msg, msglen, prio, NULL, -1);
}

/****************************************************************************
 * Name: nxmq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  This is an internal OS interface.  It is functionally
 *   equivalent to mq_send() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_send() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes  - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_send() for the list list valid return values).
 *
 ****************************************************************************/

int nxmq_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
              unsigned int prio)
{
  FAR struct file *filep;
  int ret;

  ret = fs_getfilep(mqdes, &filep);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_mq_timedsend_internal(filep, msg, msglen, prio, NULL, -1);
  fs_putfilep(filep);
  return ret;
}

/****************************************************************************
 * Name: mq_send
 *
 * Description:
 *   This function adds the specified message (msg) to the message queue
 *   (mqdes).  The "msglen" parameter specifies the length of the message
 *   in bytes pointed to by "msg."  This length must not exceed the maximum
 *   message length from the mq_getattr().
 *
 *   If the message queue is not full, mq_send() place the message in the
 *   message queue at the position indicated by the "prio" argument.
 *   Messages with higher priority will be inserted before lower priority
 *   messages.  The value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK is not set in the
 *   message queue, then mq_send() will block until space becomes available
 *   to the queue the message.
 *
 *   If the message queue is full and O_NONBLOCK is set, the message is not
 *   queued and ERROR is returned.
 *
 * Input Parameters:
 *   mqdes  - Message queue descriptor
 *   msg    - Message to send
 *   msglen - The length of the message in bytes
 *   prio   - The priority of the message
 *
 * Returned Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was full and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EBADF    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int mq_send(mqd_t mqdes, FAR const char *msg, size_t msglen,
            unsigned int prio)
{
  int ret;

  /* mq_send() is a cancellation point */

  enter_cancellation_point();

  /* Let nxmq_send() do all of the work */

  ret = nxmq_send(mqdes, msg, msglen, prio);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
