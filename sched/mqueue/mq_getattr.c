/****************************************************************************
 * sched/mqueue/mq_getattr.c
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
#include <nuttx/mqueue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mq_getattr
 *
 * Description:
 *   This functions gets status information and attributes
 *   associated with the specified message queue.
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor
 *   mq_stat - Buffer in which to return attributes
 *
 * Returned Value:
 *   0 (OK) if attributes provided, -1 (ERROR) otherwise.
 *
 * Assumptions:
 *
 ****************************************************************************/

int mq_getattr(mqd_t mqdes, struct mq_attr *mq_stat)
{
  int ret = ERROR;

  if (mqdes && mq_stat)
    {
      /* Return the attributes */

      mq_stat->mq_maxmsg  = mqdes->msgq->maxmsgs;
      mq_stat->mq_msgsize = mqdes->msgq->maxmsgsize;
      mq_stat->mq_flags   = mqdes->oflags;
      mq_stat->mq_curmsgs = mqdes->msgq->nmsgs;

      ret = OK;
    }

  return ret;
}
