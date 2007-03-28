/************************************************************
 * mq_send.c
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

#include  <nuttx/compiler.h>
#include  <nuttx/kmalloc.h>
#include  <sys/types.h>      /* uint32, etc. */
#include  <fcntl.h>
#include  <mqueue.h>
#include  <string.h>
#include  <errno.h>
#include  <sched.h>
#include  <debug.h>
#include  <nuttx/arch.h>
#include  "os_internal.h"
#ifndef CONFIG_DISABLE_SIGNALS
# include "sig_internal.h"
#endif
#include  "mq_internal.h"

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
 * Function: mq_msgalloc
 *
 * Description:
 *   The mq_msgalloc function will get a free message for use
 *   by the operating system.  The message will be allocated
 *   from the g_msgfree list.
 *
 *   If the list is empty AND the message is NOT being
 *   allocated from the interrupt level, then the message
 *   will be allocated.  If a message cannot be obtained,
 *   the operating system is dead and therefore cannot
 *   continue.
 *
 *   If the list is empty AND the message IS being allocated
 *   from the interrupt level.  This function will attempt to
 *   get a message from the g_msgfreeirq list.  If this is
 *   unsuccessful, the calling interrupt handler will be
 *   notified.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   A reference to the allocated msg structure
 *
 ************************************************************/

FAR mqmsg_t *mq_msgalloc(void)
{
  FAR mqmsg_t *mqmsg;
  irqstate_t   saved_state;

  /* If we were called from an interrupt handler, then try to
   * get the message from generally available list of messages.
   * If this fails, then try the list of messages reserved for
   * interrupt handlers
   */

  if (up_interrupt_context())
    {
      /* Try the general free list */

      mqmsg = (FAR mqmsg_t*)sq_remfirst(&g_msgfree);
      if (!mqmsg)
        {
          /* Try the free list reserved for interrupt handlers */

          mqmsg = (FAR mqmsg_t*)sq_remfirst(&g_msgfreeirq);
        }
    }

  /* We were not called from an interrupt handler. */

  else
    {
      /* Try to get the message from the generally available free list.
       * Disable interrupts -- we might be called from an interrupt handler.
       */

      saved_state = irqsave();
      mqmsg = (FAR mqmsg_t*)sq_remfirst(&g_msgfree);
      irqrestore(saved_state);

      /* If we cannot a message from the free list, then we will have to allocate one. */

      if (!mqmsg)
        {
          mqmsg = (FAR mqmsg_t *)kmalloc((sizeof (mqmsg_t)));

          /* Check if we got an allocated message */

          if (mqmsg)
            {
              mqmsg->type = MQ_ALLOC_DYN;
            }

          /* No?  We are dead */

          else
            {
              dbg("Out of messages\n");
              PANIC((uint32)OSERR_OUTOFMESSAGES);
            }
        }
    }

  return(mqmsg);
}

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  mq_send
 *
 * Description:
 *   This function adds the specificied message (msg) to the
 *   message queue (mqdes).  The "msglen" parameter specifies
 *   the length of the message in bytes pointed to by "msg."
 *   This length must not exceed the maximum message length
 *   from the mq_getattr().
 *
 *   If the message queue is not full, mq_send() will in the
 *   message in the message queue at the position indicated
 *   by the "prio" argrument.  Messages with higher priority
 *   will be inserted before lower priority messages.  The
 *   value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK
 *   is not set in the message queue, then mq_send() will
 *   block until space becomes available to the queue the
 *   message.
 *
 *   If the message queue is full and O_NONBLOCK is set,
 *   the message is not queued and ERROR is returned.
 * 
 * Parameters:
 *   mqdes - Message queue descriptor
 *   msg - Message to send
 *   msglen - The length of the message in bytes
 *   prio - The priority of the message
 *
 * Return Value:
 *   On success, mq_send() returns0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set  to  indicate the error:
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was
 *            set for the message queue description referred to
 *            by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio
 *            is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute
 *            of the message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ************************************************************/

int mq_send(mqd_t mqdes, const void *msg, size_t msglen, int prio)
{
  FAR _TCB    *rtcb;
  FAR _TCB    *btcb;
  FAR msgq_t  *msgq;
  FAR mqmsg_t *curr;
  FAR mqmsg_t *next;
  FAR mqmsg_t *prev;
  irqstate_t   saved_state;
  int          ret = ERROR;

  /* Verify the input parameters */

  if (!msg || !mqdes || prio < 0 || prio > MQ_PRIO_MAX)
    {
      *get_errno_ptr() = EINVAL;
      return ERROR;
    }

  if ((mqdes->oflags & O_WROK) == 0)
    {
      *get_errno_ptr() = EPERM;
      return ERROR;
    }

  if (msglen < 0 || msglen > (size_t)mqdes->msgq->maxmsgsize)
    {
      *get_errno_ptr() = EMSGSIZE;
      return ERROR;
    }

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* If we are sending a message from an interrupt handler, then
   * try to get message structure unconditionally.
   */

  saved_state = irqsave();
  if (up_interrupt_context())
    {
      curr = mq_msgalloc();
    }

  /* Otherwise, arbitrarily limit the number of messages in the
   * queue to the value determined when the message queue was opened.
   * This makes us more POSIX-like as well as prohibits one slow
   * responding task from consuming all available memory.
   */

  else if (msgq->nmsgs >= msgq->maxmsgs)
    {
      /* Should we block until there is sufficient space in the
       * message queue?
       */

      if ((mqdes->oflags & O_NONBLOCK) != 0)
        {
          /* No... We will return an error to the caller. */

          *get_errno_ptr() = EAGAIN;
          curr = NULL;
        }

      /* Yes... We will not return control until the message queue is
       * available.
       */

      else
        {
          boolean interrupted = FALSE;

          /* Loop until there are fewer than max allowable messages in the 
           * receiving message queue
           */

          while (msgq->nmsgs >= msgq->maxmsgs)
            {
              /* Block until the message queue is no longer full.
               * When we are unblocked, we will try again
               */

              rtcb = (FAR _TCB*)g_readytorun.head;
              rtcb->msgwaitq = msgq;
              (msgq->nwaitnotfull)++;

              *get_errno_ptr() = OK;
              up_block_task(rtcb, TSTATE_WAIT_MQNOTFULL);

              /* When we resume at this point, either (1) the message queue
               * is no longer empty, or (2) the wait has been interrupted by
               * a signal.  We can detect the latter case be examining the
               * errno value (should be EINTR).
               */

              if (*get_errno_ptr() != OK)
                {
                   interrupted = TRUE;
                   break;
                }
            }

          /* If we were not interrupted, then it should be okay to add
           * a message to the receiving message queue now.
           */

          if (!interrupted)
            {
              curr = mq_msgalloc();
            }
        }
    }

  /* We are not in an interrupt handler and the receiving message queue
   * is not full
   */

  else
    {
      /* Just allocate a message */

      curr = mq_msgalloc();
    }
  irqrestore(saved_state);

  /* Check if we were able to get a message structure */

  if (curr)
    {
      /* Construct the current message header info */

      curr->priority = (ubyte)prio;
      curr->msglen   = (ubyte)msglen;

      /* Copy the message data into the message */

      memcpy((void*)curr->mail, (const void*)msg, msglen);

      /* Insert the new message in the message queue */

      saved_state = irqsave();

      /* Search the message list to find the location to insert the new
       * message. Each is list is maintained in ascending priority order.
       */

      for (prev = NULL, next = (FAR mqmsg_t*)msgq->msglist.head;
           next && prio <= next->priority;
           prev = next, next = next->next);

      /* Add the message at the right place */

      if (prev)
        {
          sq_addafter((FAR sq_entry_t*)prev, (FAR sq_entry_t*)curr,
                      &msgq->msglist);
        }
      else
        {
          sq_addfirst((FAR sq_entry_t*)curr, &msgq->msglist);
        }

      /* Increment the count of message in the queue */

      msgq->nmsgs++;
      irqrestore(saved_state);

      /* Check if we need to notify any tasks that are attached to the
       * message queue
       */

#ifndef CONFIG_DISABLE_SIGNALS
      if (msgq->ntmqdes)
        {
          /* Remove the message notification data from the message queue. */

#ifdef CONFIG_CAN_PASS_STRUCTS
          union sigval value      = msgq->ntvalue;
#else
          void *sival_ptr         = msgq->ntvalue.sival_ptr;
#endif
          int signo               = msgq->ntsigno;
          int pid                 = msgq->ntpid;

          /* Detach the notification */

          msgq->ntpid             = INVALID_PROCESS_ID;
          msgq->ntsigno           = 0;
          msgq->ntvalue.sival_int = 0;
          msgq->ntmqdes           = NULL;

          /* Queue the signal -- What if this returns an error? */

#ifdef CONFIG_CAN_PASS_STRUCTS
          sig_mqnotempty(pid, signo, value);
#else
          sig_mqnotempty(pid, signo, sival_ptr);
#endif
        }
#endif
      /* Check if any tasks are waiting for the MQ not empty event. */

      saved_state = irqsave();
      if (msgq->nwaitnotempty > 0)
        {
          /* Find the highest priority task that is waiting for
           * this queue to be non-empty in g_waitingformqnotempty
           * list. sched_lock() should give us sufficent protection since
           * interrupts should never cause a change in this list
           */

          for (btcb = (FAR _TCB*)g_waitingformqnotempty.head;
               btcb && btcb->msgwaitq != msgq;
               btcb = btcb->flink);

          /* If one was found, unblock it */

          if (!btcb)
            {
              PANIC(OSERR_MQNONEMPTYCOUNT);
            }
          else
            {
              btcb->msgwaitq = NULL;
              msgq->nwaitnotempty--;
              up_unblock_task(btcb);
            }
        }
      irqrestore(saved_state);
      ret = OK;
    }

  sched_unlock();
  return(ret);
}

