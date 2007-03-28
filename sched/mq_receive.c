/************************************************************
 * mq_receive.c
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
#include <stdarg.h>         /* va_list */
#include <unistd.h>
#include <fcntl.h>          /* O_NONBLOCK */
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <mqueue.h>
#include <sched.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/os_external.h>
#include "os_internal.h"
#include "sig_internal.h"
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
 * Function:  mq_receive
 *
 * Description:
 *   This function receives the oldest of the highest
 *   priority messages from the message queue specified by
 *   "mqdes."  If the size of the buffer in bytes (msglen) is
 *   less than the "mq_msgsize" attribute of the message
 *   queue, mq_receive will return an error.  Otherwise, the
 *   selected message is removed from the queue and copied to
 *   "msg."
 *
 *   If the message queue is empty and O_NONBLOCK was not
 *   set, mq_receive() will block until a message is added
 *   to the message queue.  If more than one task is waiting
 *   to receive a message, only the task with the highest
 *   priority that has waited the longest will be unblocked.
 *
 *   If the queue is empty and O_NONBLOCK is set, ERROR will
 *   be returned.
 *
 * Parameters:
 *   mqdes - Message Queue Descriptor
 *   msg - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *   prio - If not NULL, the location to store message priority.
 *
 * Return Value:
 *   One success, the length of the selected message in bytes.is
 *   returned.  On failure, -1 (ERROR) is returned and the errno
 *   is set appropriately:
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set
 *            for the message queue description referred to by 'mqdes'.
 *   EPERM    Message queue opened not opened for reading.
 *   EMSGSIZE 'msglen' was less than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *   EINVAL   Invalid 'msg' or 'mqdes'
 *
 * Assumptions:
 *
 ************************************************************/

int mq_receive(mqd_t mqdes, void *msg, size_t msglen, int *prio)
{
  FAR _TCB    *rtcb;
  FAR _TCB    *btcb;
  FAR msgq_t  *msgq;
  FAR mqmsg_t *curr;
  irqstate_t   saved_state;
  ubyte        rcvmsglen;
  int          ret = ERROR;

  /* Verify the input parameters */

  if (!msg || !mqdes)
    {
      *get_errno_ptr() = EINVAL;
      return ERROR;
    }

  if ((mqdes->oflags & O_RDOK) == 0)
    {
      *get_errno_ptr() = EPERM;
      return ERROR;
    }

  if (msglen < (size_t)mqdes->msgq->maxmsgsize)
    {
      *get_errno_ptr() = EMSGSIZE;
      return ERROR;
    }

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* Several operations must be performed below:  We must determine if
   * a message is pending and, if not, wait for the message.  Since
   * messages can be sent from the interrupt level, there is a race
   * condition that can only be eliminated by disabling interrupts!
   */

  saved_state = irqsave();

  /* Get the message from the head of the queue */

  while ((curr = (FAR mqmsg_t*)sq_remfirst(&msgq->msglist)) == NULL)
    {
      /* Should we block until there the above condition has been
       * satisfied?
       */

      if (!(mqdes->oflags & O_NONBLOCK))
        {
          /* Block and try again */

          rtcb = (FAR _TCB*)g_readytorun.head;
          rtcb->msgwaitq = msgq;
          msgq->nwaitnotempty++;

          *get_errno_ptr() = OK;
          up_block_task(rtcb, TSTATE_WAIT_MQNOTEMPTY);

          /* When we resume at this point, either (1) the message queue
           * is no longer empty, or (2) the wait has been interrupted by
           * a signal.  We can detect the latter case be examining the
           * errno value (should be EINTR).
           */

          if (*get_errno_ptr() != OK)
            {
              break;
            }
        }
      else
        {
          /* The queue was empty, and the O_NONBLOCK flag was set for the
           * message queue description referred to by 'mqdes'.
           */

         *get_errno_ptr() = EAGAIN;
          break;
        }
    }

  /* If we got message, then decrement the number of messages in
   * the queue while we are still in the critical section
   */

  if (curr)
    {
      msgq->nmsgs--;
    }
  irqrestore(saved_state);

  /* Check (again) if we got a message from the message queue*/

  if (curr)
    {
      /* Get the length of the message (also the return value) */

      ret = rcvmsglen = curr->msglen;

      /* Copy the message into the caller's buffer */

      memcpy(msg, (const void*)curr->mail, rcvmsglen);

      /* Copy the message priority as well (if a buffer is provided) */

      if (prio)
        {
          *prio = curr->priority;
        }

      /* We are done with the message.  Deallocate it now. */

      mq_msgfree(curr);

      /* Check if any tasks are waiting for the MQ not full event. */

      if (msgq->nwaitnotfull > 0)
        {
          /* Find the highest priority task that is waiting for
           * this queue to be not-full in g_waitingformqnotfull list.
           * This must be performed in a critical section because
           * messages can be sent from interrupt handlers.
           */

          saved_state = irqsave();
          for (btcb = (FAR _TCB*)g_waitingformqnotfull.head;
               btcb && btcb->msgwaitq != msgq;
               btcb = btcb->flink);

          /* If one was found, unblock it.  NOTE:  There is a race
           * condition here:  the queue might be full again by the
           * time the task is unblocked
           */

          if (!btcb)
            {
              PANIC(OSERR_MQNOTFULLCOUNT);
            }
          else
            {
              btcb->msgwaitq = NULL;
              msgq->nwaitnotfull--;
              up_unblock_task(btcb);
            }
          irqrestore(saved_state);
        }
    }

  sched_unlock();
  return ret;
}
