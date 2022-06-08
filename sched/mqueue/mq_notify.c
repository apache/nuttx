/****************************************************************************
 * sched/mqueue/mq_notify.c
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

#include <mqueue.h>
#include <sched.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_notify
 *
 * Description:
 *   If "notification" is not NULL, this function connects the task with
 *   the message queue such that the specified signal will be sent to the
 *   task whenever the message changes from empty to non-empty.  Only one
 *   notification can be attached to a message queue.
 *
 *   If "notification" is NULL, the attached notification is detached (if
 *   it was held by the calling task) and the queue is available to attach
 *   another notification.
 *
 *   When the notification is sent to the registered process, its
 *   registration will be removed.  The message queue will then be
 *   available for registration.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor
 *   notification - Real-time signal structure containing:
 *      sigev_notify - Should be SIGEV_SIGNAL or SIGEV_THREAD
 *      sigev_signo - The signo to use for the notification
 *      sigev_value - Value associated with the signal
 *
 * Returned Value:
 *   On success mq_notify() returns 0; on error, -1 is returned, with
 *   errno set to indicate the error.
 *
 *   EBADF The descriptor specified in mqdes is invalid.
 *   EBUSY Another process has already registered to receive notification
 *     for this message queue.
 *   EINVAL sevp->sigev_notify is not one of the permitted values; or
 *     sevp->sigev_notify is SIGEV_SIGNAL and sevp->sigev_signo is not a
 *     valid signal number.
 *   ENOMEM
 *     Insufficient memory.
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   int mq_notify(mqd_t mqdes, const struct sigevent *notification);
 *
 *   The notification will be sent to the registered task even if another
 *   task is waiting for the message queue to become non-empty.  This is
 *   inconsistent with the POSIX specification which says, "If a process
 *   has registered for notification of message a arrival at a message
 *   queue and some process is blocked in [nx]mq_receive() waiting to receive
 *   a message when a message arrives at the queue, the arriving message
 *   message shall satisfy [nx]mq_receive()... The resulting behavior is as
 *   if the message queue remains empty, and no notification shall be sent."
 *
 ****************************************************************************/

int mq_notify(mqd_t mqdes, FAR const struct sigevent *notification)
{
  FAR struct mqueue_inode_s *msgq;
  FAR struct inode *inode;
  FAR struct file *filep;
  FAR struct tcb_s *rtcb;
  irqstate_t flags;
  int errval;

  errval = fs_getfilep(mqdes, &filep);
  if (errval < 0)
    {
      errval = -errval;
      goto errout_without_lock;
    }

  inode = filep->f_inode;

  /* Was a valid message queue descriptor provided? */

  if (!inode || !inode->i_private)
    {
      /* No.. return EBADF */

      errval = EBADF;
      goto errout_without_lock;
    }

  /* Get a pointer to the message queue */

  flags = enter_critical_section();

  /* Get the current process ID */

  rtcb = this_task();

  /* Is there already a notification attached */

  msgq = inode->i_private;
  if (msgq->ntpid == INVALID_PROCESS_ID)
    {
      /* No... Have we been asked to establish one? */

      if (notification)
        {
          /* Yes... Was a valid signal number supplied? */

          if (!GOOD_SIGNO(notification->sigev_signo))
            {
              /* No... Return EINVAL */

              errval = EINVAL;
              goto errout;
            }

          /* Yes... Assign it to the current task. */

          memcpy(&msgq->ntevent, notification,
                 sizeof(struct sigevent));

          msgq->ntpid = rtcb->pid;
        }
    }

  /* Yes... a notification is attached.  Does this task own it?
   * Is it trying to remove it?
   */

  else if ((msgq->ntpid != rtcb->pid) || (notification != NULL))
    {
      /* This thread does not own the notification OR it is
       * not trying to remove it.  Return EBUSY.
       */

      errval = EBUSY;
      goto errout;
    }
  else
    {
      /* Yes, the notification belongs to this thread.  Allow the
       * thread to detach the notification.
       */

      memset(&msgq->ntevent, 0, sizeof(struct sigevent));
      msgq->ntpid = INVALID_PROCESS_ID;
      nxsig_cancel_notification(&msgq->ntwork);
    }

  leave_critical_section(flags);
  return OK;

errout:
  leave_critical_section(flags);

errout_without_lock:
  set_errno(errval);
  return ERROR;
}
