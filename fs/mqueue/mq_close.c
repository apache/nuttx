/****************************************************************************
 *  fs/mqueue/mq_close.c
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

#include <sched.h>
#include <mqueue.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/mqueue.h>

#include "inode/inode.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_close_group
 *
 * Description:
 *   This function is used to indicate that all threads in the group are
 *   finished with the specified message queue mqdes.  The nxmq_close_group()
 *   deallocates any system resources allocated by the system for use by
 *   this task for its message queue.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Returned Value:
 *   Zero (OK) if the message queue is closed successfully.  Otherwise, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

int nxmq_close_group(mqd_t mqdes, FAR struct task_group_s *group)
{
  FAR struct mqueue_inode_s *msgq;
  FAR struct inode *inode;
  int ret = OK;

  DEBUGASSERT(mqdes != NULL && group != NULL);

  /* Verify the inputs */

  if (mqdes)
    {
      sched_lock();

      /* Find the message queue associated with the message descriptor */

      msgq = mqdes->msgq;
      DEBUGASSERT(msgq && msgq->inode);

      /* Close/free the message descriptor */

      ret = nxmq_desclose_group(mqdes, group);
      if (ret >= 0)
        {
          /* Get the inode from the message queue structure */

          inode = msgq->inode;
          DEBUGASSERT(inode->u.i_mqueue == msgq);

          /* Decrement the reference count on the inode, possibly free it */

          mq_inode_release(inode);
        }

      sched_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: nxmq_close
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_close() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_close() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_close(mqd_t mqdes)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)nxsched_self();
  int ret;

  /* Lock the scheduler to prevent any asynchronous task delete operation
   * (unlikely).
   */

  sched_lock();

  DEBUGASSERT(mqdes != NULL && rtcb != NULL && rtcb->group != NULL);

  /* Then perform the close operation */

  ret = nxmq_close_group(mqdes, rtcb->group);

  sched_unlock();
  return ret;
}

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
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Returned Value:
 *   0 (OK) if the message queue is closed successfully,
 *   otherwise, -1 (ERROR).
 *
 * Assumptions:
 * - The behavior of a task that is blocked on either a [nx]mq_send() or
 *   [nx]mq_receive() is undefined when mq_close() is called.
 * - The results of using this message queue descriptor after a successful
 *   return from mq_close() is undefined.
 *
 ****************************************************************************/

int mq_close(mqd_t mqdes)
{
  int ret;

  ret = nxmq_close(mqdes);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: mq_inode_release
 *
 * Description:
 *   Release a reference count on a message queue inode.
 *
 * Input Parameters:
 *   inode - The message queue inode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mq_inode_release(FAR struct inode *inode)
{
  int ret;

  /* Decrement the reference count on the inode */

  do
    {
      ret = inode_semtake();

      /* The only error that is expected is due to thread cancellation.
       * At this point, we must continue to free the mqueue anyway.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);

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

      nxmq_free_msgq(msgq);
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
