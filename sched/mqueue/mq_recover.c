/****************************************************************************
 * sched/mqueue/mq_recover.c
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

#include <assert.h>

#include <nuttx/mqueue.h>
#include <nuttx/sched.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_recover
 *
 * Description:
 *   This function is called when a task is deleted via task_deleted or
 *   via pthread_cancel. I checks if the task was waiting for a message
 *   queue event and adjusts counts appropriately.
 *
 * Input Parameters:
 *   tcb - The TCB of the terminated task or thread
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

void nxmq_recover(FAR struct tcb_s *tcb)
{
  FAR struct mqueue_inode_s *msgq;

  /* If were were waiting for a timed message queue event, then the
   * timer was canceled and deleted in nxtask_recover() before this
   * function was called.
   */

  /* Was the task waiting for a message queue to become non-empty? */

  msgq = tcb->waitobj;

  if (tcb->task_state == TSTATE_WAIT_MQNOTEMPTY)
    {
      /* Decrement the count of waiters */

      DEBUGASSERT(msgq && msgq->cmn.nwaitnotempty > 0);
      msgq->cmn.nwaitnotempty--;
    }

  /* Was the task waiting for a message queue to become non-full? */

  else if (tcb->task_state == TSTATE_WAIT_MQNOTFULL)
    {
      /* Decrement the count of waiters */

      DEBUGASSERT(msgq && msgq->cmn.nwaitnotfull > 0);
      msgq->cmn.nwaitnotfull--;
    }
}
