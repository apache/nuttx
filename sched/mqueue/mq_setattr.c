/****************************************************************************
 * sched/mqueue/mq_setattr.c
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

#include <fcntl.h>          /* O_NONBLOCK */
#include <mqueue.h>

#include <nuttx/mqueue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mq_setattr
 *
 * Description:
 *   This function sets the attributes associated with the
 *   specified message queue "mqdes."  Only the "O_NONBLOCK"
 *   bit of the "mq_flags" can be changed.
 *
 *   If "oldstat" is non-null, mq_setattr() will store the
 *   previous message queue attributes at that location (just
 *   as would have been returned by mq_getattr()).
 *
 * Input Parameters:
 *   mqdes - Message queue descriptor
 *   mq_stat - New attributes
 *   oldstate - Old attributes
 *
 * Returned Value:
 *   0 (OK) if attributes are set successfully, otherwise
 *   -1 (ERROR).
 *
 * Assumptions:
 *
 ****************************************************************************/

int mq_setattr(mqd_t mqdes, const struct mq_attr *mq_stat,
               struct mq_attr *oldstat)
{
  int ret = ERROR;

  if (mqdes && mq_stat)
    {
      /* Return the attributes if so requested */

      if (oldstat)
        {
          mq_getattr(mqdes, oldstat);
        }

      /* Set the new value of the O_NONBLOCK flag. */

      mqdes->oflags = ((mq_stat->mq_flags & O_NONBLOCK) |
                       (mqdes->oflags & (~O_NONBLOCK)));
      ret = OK;
    }

  return ret;
}
