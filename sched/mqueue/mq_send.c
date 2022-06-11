/****************************************************************************
 * sched/mqueue/mq_send.c
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

#include  <nuttx/config.h>

#include  <sys/types.h>
#include  <mqueue.h>
#include  <errno.h>
#include  <debug.h>

#include  <nuttx/irq.h>
#include  <nuttx/arch.h>
#include  <nuttx/cancelpt.h>

#include  "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  FAR struct inode *inode = mq->f_inode;
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *mqmsg;
  irqstate_t flags;
  int ret;

  if (!inode)
    {
      return -EBADF;
    }

  msgq = inode->i_private;

  /* Verify the input parameters -- setting errno appropriately
   * on any failures to verify.
   */

  ret = nxmq_verify_send(msgq, mq->f_oflags, msg, msglen, prio);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a message structure:
   * - Immediately if we are called from an interrupt handler.
   * - Immediately if the message queue is not full, or
   * - After successfully waiting for the message queue to become
   *   non-FULL.  This would fail with EAGAIN, EINTR, or ETIMEOUT.
   */

  flags = enter_critical_section();

  if (!up_interrupt_context())           /* In an interrupt handler? */
    {
      /* No.. Not in an interrupt handler.  Is the message queue FULL? */

      if (msgq->nmsgs >= msgq->maxmsgs)
        {
          /* Yes.. the message queue is full.  Wait for space to become
           * available in the message queue.
           */

          ret = nxmq_wait_send(msgq, mq->f_oflags);
        }
    }

  /* ret can only be negative if nxmq_wait_send failed */

  if (ret == OK)
    {
      /* Now allocate the message. */

      mqmsg = nxmq_alloc_msg();

      /* Check if the message was successfully allocated */

      /* The allocation was successful (implying that we can also send the
       * message). Perform the message send.
       *
       * NOTE: There is a race condition here: What if a message is added by
       * interrupt related logic so that queue again becomes non-empty.
       * That is handled because nxmq_do_send() will permit the maxmsgs limit
       * to be exceeded in that case.
       */

      ret = (mqmsg == NULL) ? -ENOMEM :
            nxmq_do_send(msgq, mqmsg, msg, msglen, prio);
    }

  leave_critical_section(flags);

  return ret;
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

  return file_mq_send(filep, msg, msglen, prio);
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
 *   EPERM    Message queue opened not opened for writing.
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
