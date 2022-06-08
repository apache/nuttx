/****************************************************************************
 * sched/mqueue/mq_receive.c
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
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <mqueue.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mqueue.h>
#include <nuttx/cancelpt.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_mq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mq."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_receive() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq     - Message Queue Descriptor
 *   msg    - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *   prio   - If not NULL, the location to store message priority.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_receive() for the list list valid return values).
 *
 ****************************************************************************/

ssize_t file_mq_receive(FAR struct file *mq, FAR char *msg, size_t msglen,
                        FAR unsigned int *prio)
{
  FAR struct inode *inode = mq->f_inode;
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *mqmsg;
  irqstate_t flags;
  ssize_t ret;

  if (!inode)
    {
      return -EBADF;
    }

  msgq = inode->i_private;

  DEBUGASSERT(up_interrupt_context() == false);

  /* Verify the input parameters and, in case of an error, set
   * errno appropriately.
   */

  ret = nxmq_verify_receive(msgq, mq->f_oflags, msg, msglen);
  if (ret < 0)
    {
      return ret;
    }

  /* Furthermore, nxmq_wait_receive() expects to have interrupts disabled
   * because messages can be sent from interrupt level.
   */

  flags = enter_critical_section();

  /* Get the message from the message queue */

  ret = nxmq_wait_receive(msgq, mq->f_oflags, &mqmsg);

  /* Check if we got a message from the message queue.  We might
   * not have a message if:
   *
   * - The message queue is empty and O_NONBLOCK is set in the mq
   * - The wait was interrupted by a signal
   */

  if (ret == OK)
    {
      ret = nxmq_do_receive(msgq, mqmsg, msg, prio);
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nxmq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mqdes."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_receive() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes  - Message Queue Descriptor
 *   msg    - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *   prio   - If not NULL, the location to store message priority.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   (see mq_receive() for the list list valid return values).
 *
 ****************************************************************************/

ssize_t nxmq_receive(mqd_t mqdes, FAR char *msg, size_t msglen,
                     FAR unsigned int *prio)
{
  FAR struct file *filep;
  int ret;

  ret = fs_getfilep(mqdes, &filep);
  if (ret < 0)
    {
      return ret;
    }

  return file_mq_receive(filep, msg, msglen, prio);
}

/****************************************************************************
 * Name: mq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mqdes."  If the size of the
 *   buffer in bytes (msglen) is less than the "mq_msgsize" attribute of
 *   the message queue, mq_receive will return an error.  Otherwise, the
 *   selected message is removed from the queue and copied to "msg."
 *
 *   If the message queue is empty and O_NONBLOCK was not set,
 *   mq_receive() will block until a message is added to the message
 *   queue.  If more than one task is waiting to receive a message, only
 *   the task with the highest priority that has waited the longest will
 *   be unblocked.
 *
 *   If the queue is empty and O_NONBLOCK is set, ERROR will be returned.
 *
 * Input Parameters:
 *   mqdes  - Message Queue Descriptor
 *   msg    - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *   prio   - If not NULL, the location to store message priority.
 *
 * Returned Value:
 *   On success, the length of the selected message in bytes is returned.
 *   On failure, -1 (ERROR) is returned and the errno is set appropriately:
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set
 *            for the message queue description referred to by 'mqdes'.
 *   EPERM    Message queue opened not opened for reading.
 *   EMSGSIZE 'msglen' was less than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *   EINVAL   Invalid 'msg' or 'mqdes'
 *
 ****************************************************************************/

ssize_t mq_receive(mqd_t mqdes, FAR char *msg, size_t msglen,
                   FAR unsigned int *prio)
{
  int ret;

  /* mq_receive() is a cancellation point */

  enter_cancellation_point();

  /* Let nxmq_receive do all of the work */

  ret = nxmq_receive(mqdes, msg, msglen, prio);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
