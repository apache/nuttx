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

#include <errno.h>
#include <fcntl.h>          /* O_NONBLOCK */
#include <mqueue.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mqueue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  file_mq_setattr
 *
 * Description:
 *   This function sets the attributes associated with the
 *   specified message queue "mq".  Only the "O_NONBLOCK"
 *   bit of the "mq_flags" can be changed.
 *
 *   If "oldstat" is non-null, mq_setattr() will store the
 *   previous message queue attributes at that location (just
 *   as would have been returned by file_mq_getattr()).
 *
 * Input Parameters:
 *   mq - Message queue descriptor
 *   mq_stat - New attributes
 *   oldstate - Old attributes
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int file_mq_setattr(FAR struct file *mq, FAR const struct mq_attr *mq_stat,
                    FAR struct mq_attr *oldstat)
{
  if (!mq || !mq_stat)
    {
      return -EINVAL;
    }

  /* Return the attributes if so requested */

  if (oldstat)
    {
      file_mq_getattr(mq, oldstat);
    }

  /* Set the new value of the O_NONBLOCK flag. */

  mq->f_oflags = ((mq_stat->mq_flags & O_NONBLOCK) |
                 (mq->f_oflags & (~O_NONBLOCK)));

  return 0;
}

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
  FAR struct file *filep;
  int ret;

  ret = fs_getfilep(mqdes, &filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  ret = file_mq_setattr(filep, mq_stat, oldstat);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
