/****************************************************************************
 *  sched/mqueue/mq_msgqfree.c
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

#include <debug.h>
#include <nuttx/kmalloc.h>
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_free_msgq
 *
 * Description:
 *   This function deallocates an initialized message queue structure.
 *   First, it deallocates all of the queued messages in the message
 *   queue.  It is assumed that this message is fully unlinked and
 *   closed so that no thread will attempt access it while it is being
 *   deleted.
 *
 * Input Parameters:
 *   msgq - Named essage queue to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_free_msgq(FAR struct mqueue_inode_s *msgq)
{
  FAR struct mqueue_msg_s *curr;
  FAR struct mqueue_msg_s *next;

  /* Deallocate any stranded messages in the message queue. */

  curr = (FAR struct mqueue_msg_s *)msgq->msglist.head;
  while (curr)
    {
      /* Deallocate the message structure. */

      next = curr->next;
      nxmq_free_msg(curr);
      curr = next;
    }

  /* Then deallocate the message queue itself */

  kmm_free(msgq);
}
