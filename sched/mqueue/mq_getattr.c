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

#include <errno.h>
#include <mqueue.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mqueue.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  file_mq_getattr
 *
 * Description:
 *   This functions gets status information and attributes
 *   associated with the specified message queue.
 *
 * Input Parameters:
 *   mq      - Message queue descriptor
 *   mq_stat - Buffer in which to return attributes
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int file_mq_getattr(FAR struct file *mq, FAR struct mq_attr *mq_stat)
{
  FAR struct mqueue_inode_s *msgq;
  FAR struct inode *inode;

  if (!mq || !mq_stat)
    {
      return -EINVAL;
    }

  inode = mq->f_inode;
  if (!inode)
    {
      return -EBADF;
    }

  msgq = inode->i_private;

  mq_stat->mq_maxmsg  = msgq->maxmsgs;
  mq_stat->mq_msgsize = msgq->maxmsgsize;
  mq_stat->mq_flags   = mq->f_oflags;
  mq_stat->mq_curmsgs = msgq->nmsgs;

  return 0;
}

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
  FAR struct file *filep;
  int ret;

  ret = fs_getfilep(mqdes, &filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  ret = file_mq_getattr(filep, mq_stat);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
