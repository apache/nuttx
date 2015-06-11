/****************************************************************************
 *  fs/mqueue/mq_close.c
 *
 *   Copyright (C) 2007, 2009, 2013-2014 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <mqueue.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>

#include "inode/inode.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_close
 *
 * Description:
 *   This function is used to indicate that the calling task is finished
 *   with the specified message queue mqdes.  The mq_close() deallocates
 *   any system resources allocated by the system for use by this task for
 *   its message queue.
 *
 *   If the calling task has attached a notification to the message queue
 *   via this mqdes, this attachment will be removed and the message queue
 *   is available for another process to attach a notification.
 *
 * Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Return Value:
 *   0 (OK) if the message queue is closed successfully,
 *   otherwise, -1 (ERROR).
 *
 * Assumptions:
 * - The behavior of a task that is blocked on either a mq_send() or
 *   mq_receive() is undefined when mq_close() is called.
 * - The results of using this message queue descriptor after a successful
 *   return from mq_close() is undefined.
 *
 ****************************************************************************/

int mq_close(mqd_t mqdes)
{
  FAR struct mqueue_inode_s *msgq;
  FAR struct inode *inode;

  /* Verify the inputs */

   if (mqdes)
     {
       sched_lock();

       /* Find the message queue associated with the message descriptor */

       msgq = mqdes->msgq;
       DEBUGASSERT(msgq && msgq->inode);

       /* Close/free the message descriptor */

       mq_desclose(mqdes);

       /* Get the inode from the message queue structure */

       inode = msgq->inode;
       DEBUGASSERT(inode->u.i_mqueue == msgq);

       /* Decrement the reference count on the inode, possibly freeing it */

       mq_inode_release(inode);
       sched_unlock();
     }

  return OK;
}

/****************************************************************************
 * Name: mq_close
 *
 * Description:
 *   Release a reference count on a message queue inode.
 *
 * Parameters:
 *   inode - The message queue inode
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void mq_inode_release(FAR struct inode *inode)
{
  /* Decrement the reference count on the inode */

  inode_semtake();
  if (inode->i_crefs > 0)
    {
      inode->i_crefs--;
    }

  /* If the message queue was previously unlinked and the reference count
   * has decremented to zero, then release the message queue and delete
   * the inode now.
   */

   if (inode->i_crefs <= 0 && (inode->i_flags & FSNODEFLAG_DELETED) != 0)
     {
       FAR struct mqueue_inode_s *msgq = inode->u.i_mqueue;
       DEBUGASSERT(msgq);

       /* Free the message queue (and any messages left in it) */

       mq_msgqfree(msgq);
       inode->u.i_mqueue = NULL;

       /* Release and free the inode container.  If it has been properly
        * unlinked, then the peer pointer should be NULL.
        */

       inode_semgive();

       DEBUGASSERT(inode->i_peer == NULL);
       inode_free(inode);
       return;
    }

  inode_semgive();
}
