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
#include <string.h>
#include <assert.h>
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
 *   select message is removed from the queue and copied to
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
 *   prio - If not NULL, the location to store message
 *      priority.
 *
 * Return Value:
 *   Length of the selected message in bytes, otherwise -1
 *   (ERROR).
 *
 * Assumptions:
 *
 ************************************************************/

int mq_receive(mqd_t mqdes, void *msg, size_t msglen, int *prio)
{
  _TCB      *rtcb;
  _TCB      *btcb;
  msgq_t    *msgq;
  mqmsg_t   *curr;
  irqstate_t saved_state;
  ubyte      rcvmsglen;
  int        ret = ERROR;

  /* Verify the input parameters */

  sched_lock();
  if (msg && mqdes && (mqdes->oflags & O_RDOK) != 0 &&
      msglen >= (size_t)mqdes->msgq->maxmsgsize)
    {
      /* Get a pointer to the message queue */

      msgq = mqdes->msgq;

      /* Several operations must be performed below:  We must determine if
       * a message is pending and, if not, wait for the message.  Since
       * messages can be sent from the interrupt level, there is a race
       * condition that can only be eliminated by disabling interrupts!
       */

      saved_state = irqsave();

      /* Get the message from the head of the queue */

      while ((curr = (mqmsg_t*)sq_remfirst(&msgq->msglist)) == NULL)
        {
          /* Should we block until there the above condition has been
           * satisfied?
           */

          if (!(mqdes->oflags & O_NONBLOCK))
            {
              /* Block and try again */

              rtcb = (_TCB*)g_readytorun.head;
              rtcb->msgwaitq = msgq;
              msgq->nwaitnotempty++;
              up_block_task(rtcb, TSTATE_WAIT_MQNOTEMPTY);
            }
          else
            {
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

          memcpy((void*)curr->mail, msg, rcvmsglen);

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
              for (btcb = (_TCB*)g_waitingformqnotfull.head;
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
    }

  sched_unlock();
  return ret;
}
