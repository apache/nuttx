/****************************************************************************
 *  sched/mqueue/mq_receive.c
 *
 *   Copyright (C) 2007, 2009, 2016-2017 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
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
 * Name: nxmq_receive
 *
 * Description:
 *   This function receives the oldest of the highest priority messages
 *   from the message queue specified by "mqdes."  This is an internal OS
 *   interface.  It is functionally equivalent to mq_receive except that:
 *
 *   - It is not a cancellaction point, and
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
                     FAR int *prio)
{
  FAR struct mqueue_msg_s *mqmsg;
  irqstate_t flags;
  ssize_t ret;

  DEBUGASSERT(up_interrupt_context() == false);

  /* Verify the input parameters and, in case of an error, set
   * errno appropriately.
   */

  ret = nxmq_verify_receive(mqdes, msg, msglen);
  if (ret < 0)
    {
      return ret;
    }

  /* Get the next message from the message queue.  We will disable
   * pre-emption until we have completed the message received.  This
   * is not too bad because if the receipt takes a long time, it will
   * be because we are blocked waiting for a message and pre-emption
   * will be re-enabled while we are blocked
   */

  sched_lock();

  /* Furthermore, nxmq_wait_receive() expects to have interrupts disabled
   * because messages can be sent from interrupt level.
   */

  flags = enter_critical_section();

  /* Get the message from the message queue */

  ret = nxmq_wait_receive(mqdes, &mqmsg);
  leave_critical_section(flags);

  /* Check if we got a message from the message queue.  We might
   * not have a message if:
   *
   * - The message queue is empty and O_NONBLOCK is set in the mqdes
   * - The wait was interrupted by a signal
   */

  if (ret >= 0)
    {
      DEBUGASSERT(mqmsg != NULL);
      ret = nxmq_do_receive(mqdes, mqmsg, msg, prio);
    }

  sched_unlock();
  return ret;
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
 *   One success, the length of the selected message in bytes is returned.
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
                   FAR int *prio)
{
  int ret;

  /* mq_receive() is a cancellation point */

  (void)enter_cancellation_point();

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
