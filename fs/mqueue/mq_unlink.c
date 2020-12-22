/****************************************************************************
 * fs/mqueue/mq_unlink.c
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

#include <stdbool.h>
#include <stdio.h>
#include <mqueue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/mqueue.h>

#include "inode/inode.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_unlink
 *
 * Description:
 *   This is an internal OS interface.  It is functionally equivalent to
 *   mq_unlink() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_unlink() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success. A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_unlink(FAR const char *mq_name)
{
  char fullpath[MAX_MQUEUE_PATH];

  /* Get the full path to the message queue */

  snprintf(fullpath, MAX_MQUEUE_PATH, CONFIG_FS_MQUEUE_MPATH "/%s", mq_name);

  return nx_unlink(fullpath);
}

/****************************************************************************
 * Name: mq_unlink
 *
 * Description:
 *   This function removes the message queue named by "mq_name." If one
 *   or more tasks have the message queue open when mq_unlink() is called,
 *   removal of the message queue is postponed until all references to the
 *   message queue have been closed.
 *
 * Input Parameters:
 *   mq_name - Name of the message queue
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int mq_unlink(FAR const char *mq_name)
{
  char fullpath[MAX_MQUEUE_PATH];

  /* Get the full path to the message queue */

  snprintf(fullpath, MAX_MQUEUE_PATH, CONFIG_FS_MQUEUE_MPATH "/%s", mq_name);

  return unlink(fullpath);
}
