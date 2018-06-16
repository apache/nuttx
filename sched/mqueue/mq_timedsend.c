/****************************************************************************
 *  sched/mqueue/mq_timedsend.c
 *
 *   Copyright (C) 2007-2009, 2011, 2013-2017 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <unistd.h>
#include <mqueue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/cancelpt.h>

#include "clock/clock.h"
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
 *   argc  - the number of arguments (should be 1)
 *   pid   - the task ID of the task to wakeup
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void nxmq_sndtimeout(int argc, wdparm_t pid)
{
  FAR struct tcb_s *wtcb;
  irqstate_t flags;

  /* Disable interrupts.  This is necessary because an interrupt handler may
   * attempt to send a message while we are doing this.
   */

  flags = enter_critical_section();

  /* Get the TCB associated with this pid.  It is possible that task may no
   * longer be active when this watchdog goes off.
   */

  wtcb = sched_gettcb((pid_t)pid);

  /* It is also possible that an interrupt/context switch beat us to the
   * punch and already changed the task's state.
   */

  if (wtcb != NULL && wtcb->task_state == TSTATE_WAIT_MQNOTFULL)
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
 *   - It is not a cancellaction point, and
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
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 ****************************************************************************/

int nxmq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio,
                   FAR const struct timespec *abstime)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct mqueue_inode_s *msgq;
  FAR struct mqueue_msg_s *mqmsg = NULL;
  irqstate_t flags;
  sclock_t ticks;
  int result;
  int ret;

  DEBUGASSERT(up_interrupt_context() == false && rtcb->waitdog == NULL);

  /* Verify the input parameters on any failures to verify. */

  ret = nxmq_verify_send(mqdes, msg, msglen, prio);
  if (ret < 0)
    {
      return ret;
    }

  /* Pre-allocate a message structure */

  mqmsg = nxmq_alloc_msg();
  if (mqmsg == NULL)
    {
      /* Failed to allocate the message. nxmq_alloc_msg() does not set the
       * errno value.
       */

      return -ENOMEM;
    }

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* OpenGroup.org: "Under no circumstance shall the operation fail with a
   * timeout if there is sufficient room in the queue to add the message
   * immediately. The validity of the abstime parameter need not be checked
   * when there is sufficient room in the queue."
   *
   * Also ignore the time value if for some crazy reason we were called from
   * an interrupt handler.  This probably really should be an assertion.
   *
   * NOTE: There is a race condition here: What if a message is added by
   * interrupt related logic so that queue again becomes non-empty.  That
   * is handled because nxmq_do_send() will permit the maxmsgs limit to be
   * exceeded in that case.
   */

  if (msgq->nmsgs < msgq->maxmsgs || up_interrupt_context())
    {
      /* Do the send with no further checks (possibly exceeding maxmsgs)
       * Currently nxmq_do_send() always returns OK.
       */

      ret = nxmq_do_send(mqdes, mqmsg, msg, msglen, prio);
      sched_unlock();
      return ret;
    }

  /* The message queue is full... We are going to wait.  Now we must have a
   * valid time value.
   */

  if (!abstime || abstime->tv_nsec < 0 || abstime->tv_nsec >= 1000000000)
    {
      ret = -EINVAL;
      goto errout_with_mqmsg;
    }

  /* Create a watchdog.  We will not actually need this watchdog
   * unless the queue is full, but we will reserve it up front
   * before we enter the following critical section.
   */

  rtcb->waitdog = wd_create();
  if (!rtcb->waitdog)
    {
      ret = -EINVAL;
      goto errout_with_mqmsg;
    }

  /* We are not in an interrupt handler and the message queue is full.
   * Set up a timed wait for the message queue to become non-full.
   *
   * Convert the timespec to clock ticks.  We must have interrupts
   * disabled here so that this time stays valid until the wait begins.
   */

  flags  = enter_critical_section();
  result = clock_abstime2ticks(CLOCK_REALTIME, abstime, &ticks);

  /* If the time has already expired and the message queue is empty,
   * return immediately.
   */

  if (result == OK && ticks <= 0)
    {
      result = ETIMEDOUT;
    }

  /* Handle any time-related errors */

  if (result != OK)
    {
      ret = -result;
      goto errout_in_critical_section;
    }

  /* Start the watchdog and begin the wait for MQ not full */

  (void)wd_start(rtcb->waitdog, ticks, (wdentry_t)nxmq_sndtimeout,
                 1, getpid());

  /* And wait for the message queue to be non-empty */

  ret = nxmq_wait_send(mqdes);

  /* This may return with an error and errno set to either EINTR
   * or ETIMEOUT.  Cancel the watchdog timer in any event.
   */

  wd_cancel(rtcb->waitdog);

  /* Check if nxmq_wait_send() failed */

  if (ret < 0)
    {
      /* nxmq_wait_send() failed. */

      goto errout_in_critical_section;
    }

  /* That is the end of the atomic operations */

  leave_critical_section(flags);

  /* If any of the above failed, set the errno.  Otherwise, there should
   * be space for another message in the message queue.  NOW we can allocate
   * the message structure.
   *
   * Currently nxmq_do_send() always returns OK.
   */

  ret = nxmq_do_send(mqdes, mqmsg, msg, msglen, prio);

  sched_unlock();
  wd_delete(rtcb->waitdog);
  rtcb->waitdog = NULL;
  leave_cancellation_point();
  return ret;

  /* Exit here with (1) the scheduler locked, (2) a message allocated, (3) a
   * wdog allocated, and (4) interrupts disabled.
   */

errout_in_critical_section:
  leave_critical_section(flags);
  wd_delete(rtcb->waitdog);
  rtcb->waitdog = NULL;

  /* Exit here with (1) the scheduler locked and 2) a message allocated.  The
   * error code is in 'result'
   */

errout_with_mqmsg:
  nxmq_free_msg(mqmsg);
  sched_unlock();
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
 *   If the message queue is not full, mq_timedsend() place the message in the
 *   message queue at the position indicated by the "prio" argrument.
 *   Messages with higher priority will be inserted before lower priority
 *   messages.  The value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK is not set in the
 *   message queue, then mq_timedsend() will block until space becomes available
 *   to the queue the message or a timeout occurs.
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
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ****************************************************************************/

int mq_timedsend(mqd_t mqdes, FAR const char *msg, size_t msglen, int prio,
                 FAR const struct timespec *abstime)
{
  int ret;

  /* mq_timedsend() is a cancellation point */

  (void)enter_cancellation_point();

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
