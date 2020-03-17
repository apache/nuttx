/****************************************************************************
 *  sched/mqueue/mq_desclose.c
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
#include <assert.h>
#include <queue.h>

#include <nuttx/sched.h>
#include <nuttx/mqueue.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_desfree
 *
 * Description:
 *   Deallocate a message queue descriptor but returning it to the free list
 *
 * Input Parameters:
 *   mqdes - message queue descriptor to free
 *
 ****************************************************************************/

#define mq_desfree(mqdes) sq_addlast((FAR sq_entry_t*)mqdes, &g_desfree)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_desclose_group
 *
 * Description:
 *   This function performs the portion of the mq_close operation related
 *   to freeing resource used by the message queue descriptor itself.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   and failure.
 *
 * Assumptions:
 * - Called only from mq_close() with the scheduler locked.
 *
 ****************************************************************************/

int nxmq_desclose_group(mqd_t mqdes, FAR struct task_group_s *group)
{
  FAR struct mqueue_inode_s *msgq;
#ifdef CONFIG_DEBUG_FEATURES
  mqd_t mq_ptr;
#endif

  DEBUGASSERT(mqdes != NULL && group != NULL);

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that msgq is valid for closing.  It must be owned by the current
   * group.  NOTE the call to sq_rem() below would corrupt the descriptor
   * list if mqdes did not lie in the list.
   */

  mq_ptr = (mqd_t)sq_peek(&group->tg_msgdesq);
  while (mq_ptr)
    {
      if (mq_ptr == mqdes)
        {
           break;
        }

      mq_ptr = (mqd_t)sq_next(mq_ptr);
    }

  DEBUGASSERT(mq_ptr != NULL);
  if (mq_ptr == NULL)
    {
      /* 'mqdes' does not lie in the group's list of message descriptors. */

      return -EPERM;
    }
#endif

  /* Remove the message descriptor from the current group's list of message
   * descriptors.
   */

  sq_rem((FAR sq_entry_t *)mqdes, &group->tg_msgdesq);

  /* Find the message queue associated with the message descriptor */

  msgq = mqdes->msgq;

  /* Check if the calling task has a notification attached to the message
   * queue via this mqdes.
   */

  if (msgq->ntmqdes == mqdes)
    {
      memset(&msgq->ntevent, 0, sizeof(struct sigevent));
      msgq->ntpid   = INVALID_PROCESS_ID;
      msgq->ntmqdes = NULL;
      nxsig_cancel_notification(&msgq->ntwork);
    }

  /* Deallocate the message descriptor */

  mq_desfree(mqdes);
  return OK;
}
