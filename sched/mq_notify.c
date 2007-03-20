/************************************************************
 * mq_notify.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>      /* uint32, etc. */
#include <signal.h>
#include <mqueue.h>
#include <sched.h>
#include "os_internal.h"
#include "mq_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  mq_notify
 *
 * Description:
 *   If "notification" is not NULL, this function connects
 *   the task with the message queue such that the specified
 *   signal will be sent to the task whenever the message
 *   changes from empty to non-empty.  One one notification
 *   can be attached to a message queue.
 *
 *   If "notification" is NULL, the attached notification is
 *   detached (if it was held by the calling task) and the
 *   queue is available to attach another notification.
 *
 *   When the notification is sent to the registered process,
 *   its registration will be removed.  The message queue
 *   will then be available for registration.
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *   notification - Real-time signal structure containing:
 *      sigev_notify - Should be SIGEV_SIGNAL (but actually ignored)
 *      sigev_signo - The signo to use for the notification
 *      sigev_value - Value associated with the signal
 *
 * Return Value:
 *   None
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
 *   queue and some process is blocked in mq_receive() waiting to receive
 *   a message when a message arrives at the queue, the arriving message
 *   message shall satisfy mq_receive()... The resulting behavior is as if
 *   the message queue remains empty, and no notification shall be sent."
 *
 ************************************************************/

int mq_notify(mqd_t mqdes, const struct sigevent *notification)
{
  _TCB   *rtcb;
  msgq_t *msgq;
  int     ret = ERROR;

  if (mqdes)
    {
      sched_lock();

      /* Get a pointer to the message queue */

      msgq = mqdes->msgq;

      /* Get the current process ID */

      rtcb = (_TCB*)g_readytorun.head;

      /* Is there already a notification attached */

      if (!msgq->ntmqdes)
        {
          /* No... Have we been asked to establish one?  Make
           * sure a good signal number has been provided
           */

          if (notification && GOOD_SIGNO(notification->sigev_signo))
            {
              /* Yes... Assign it to the current task. */

              msgq->ntvalue.sival_ptr = notification->sigev_value.sival_ptr;
              msgq->ntsigno           = notification->sigev_signo;
              msgq->ntpid             = rtcb->pid;
              msgq->ntmqdes           = mqdes;
              ret                     = OK;
            }
        }

      /* Yes... a notification is attached.  Does this task own it?
       * Is it trying to remove it?
       */

      else if ((msgq->ntpid == rtcb->pid) && (!notification))
        {
          /* Yes... Detach the notification */

          msgq->ntpid             = INVALID_PROCESS_ID;
          msgq->ntsigno           = 0;
          msgq->ntvalue.sival_ptr = NULL;
          msgq->ntmqdes           = NULL;
          ret                     = OK;
        }
      sched_unlock();
    }

  return ret;
}
