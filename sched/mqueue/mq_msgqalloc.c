/****************************************************************************
 * sched/mqueue/mq_msgqalloc.c
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
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/mqueue.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_alloc_msgq
 *
 * Description:
 *   This function implements a part of the POSIX message queue open logic.
 *   It allocates and initializes a struct mqueue_inode_s structure.
 *
 * Input Parameters:
 *   attr   - The mq_maxmsg attribute is used at the time that the message
 *            queue is created to determine the maximum number of
 *            messages that may be placed in the message queue.
 *   pmsgq  - This parameter is a address of a pointer
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 *   EINVAL    attr is NULL or either attr->mq_mqssize or attr->mq_maxmsg
 *             have an invalid value
 *   ENOSPC    There is insufficient space for the creation of the new
 *             message queue
 *
 ****************************************************************************/

int nxmq_alloc_msgq(FAR struct mq_attr *attr,
                    FAR struct mqueue_inode_s **pmsgq)
{
  FAR struct mqueue_inode_s *msgq;

  /* Check if the caller is attempting to allocate a message for messages
   * larger than the configured maximum message size.
   */

  DEBUGASSERT((!attr || attr->mq_msgsize <= MQ_MAX_BYTES) && pmsgq);
  if ((attr && attr->mq_msgsize > MQ_MAX_BYTES) || !pmsgq)
    {
      return -EINVAL;
    }

  /* Allocate memory for the new message queue. */

  msgq = (FAR struct mqueue_inode_s *)
    kmm_zalloc(sizeof(struct mqueue_inode_s));

  if (msgq)
    {
      /* Initialize the new named message queue */

      sq_init(&msgq->msglist);
      if (attr)
        {
          msgq->maxmsgs    = (int16_t)attr->mq_maxmsg;
          msgq->maxmsgsize = (int16_t)attr->mq_msgsize;
        }
      else
        {
          msgq->maxmsgs    = MQ_MAX_MSGS;
          msgq->maxmsgsize = MQ_MAX_BYTES;
        }

#ifndef CONFIG_DISABLE_MQUEUE_NOTIFICATION
      msgq->ntpid = INVALID_PROCESS_ID;
#endif
    }
  else
    {
      return -ENOSPC;
    }

  *pmsgq = msgq;
  return OK;
}
