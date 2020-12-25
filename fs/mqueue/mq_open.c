/****************************************************************************
 *  fs/mqueue/mq_open.c
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
#include <stdarg.h>
#include <stdio.h>
#include <sched.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/mqueue.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task. This is an internal OS interface.  It is functionally
 *   equivalent to mq_open() except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *  See comments with mq_open() for a more complete description of the
 *  behavior of this function
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode (ignored), and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success, mqdes point to the new message queue descriptor.
 *   A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxmq_open(FAR const char *mq_name, int oflags, mode_t mode,
              FAR struct mq_attr *attr, FAR mqd_t *mqdes)
{
  FAR struct inode *inode;
  FAR struct mqueue_inode_s *msgq;
  struct inode_search_s desc;
  char fullpath[MAX_MQUEUE_PATH];
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (mq_name == NULL || *mq_name == '\0')
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Skip over any leading '/'.  All message queue paths are relative to
   * CONFIG_FS_MQUEUE_MPATH.
   */

  while (*mq_name == '/')
    {
      mq_name++;
    }

  /* Get the full path to the message queue */

  snprintf(fullpath, MAX_MQUEUE_PATH, CONFIG_FS_MQUEUE_MPATH "/%s", mq_name);

  /* Make sure that the check for the existence of the message queue
   * and the creation of the message queue are atomic with respect to
   * other processes executing mq_open().  A simple sched_lock() should
   * be sufficient.
   */

  sched_lock();

  /* Get the inode for this mqueue.  This should succeed if the message
   * queue has already been created.  In this case, inode_find() will
   * have incremented the reference count on the inode.
   */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at this path.  Get the search results */

      inode = desc.node;
      DEBUGASSERT(inode != NULL);

      /* Verify that the inode is a message queue */

      if (!INODE_IS_MQUEUE(inode))
        {
          ret = -ENXIO;
          goto errout_with_inode;
        }

      /* It exists and is a message queue.  Check if the caller wanted to
       * create a new mqueue with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          ret = -EEXIST;
          goto errout_with_inode;
        }

      /* Create a message queue descriptor for the current thread */

      msgq  = inode->u.i_mqueue;
      *mqdes = nxmq_create_des(NULL, msgq, oflags);
      if (!*mqdes)
        {
          ret = -ENOMEM;
          goto errout_with_inode;
        }
    }
  else
    {
      /* The mqueue does not exists.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The mqueue does not exist and O_CREAT is not set */

          ret = -ENOENT;
          goto errout_with_lock;
        }

      /* Create an inode in the pseudo-filesystem at this path */

      ret = inode_semtake();
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      ret = inode_reserve(fullpath, &inode);
      inode_semgive();

      if (ret < 0)
        {
          goto errout_with_lock;
        }

      /* Allocate memory for the new message queue.  The new inode will
       * be created with a reference count of zero.
       */

      msgq = (FAR struct mqueue_inode_s *)nxmq_alloc_msgq(mode, attr);
      if (!msgq)
        {
          ret = -ENOSPC;
          goto errout_with_inode;
        }

      /* Create a message queue descriptor for the TCB */

      *mqdes = nxmq_create_des(NULL, msgq, oflags);
      if (!*mqdes)
        {
          ret = -ENOMEM;
          goto errout_with_msgq;
        }

      /* Bind the message queue and the inode structure */

      INODE_SET_MQUEUE(inode);
      inode->u.i_mqueue = msgq;
      msgq->inode       = inode;

      /* Set the initial reference count on this inode to one */

      inode->i_crefs    = 1;
    }

  RELEASE_SEARCH(&desc);
  sched_unlock();
  return OK;

errout_with_msgq:
  nxmq_free_msgq(msgq);
  inode->u.i_mqueue = NULL;

errout_with_inode:
  inode_release(inode);

errout_with_lock:
  RELEASE_SEARCH(&desc);
  sched_unlock();

errout:
  return ret;
}

/****************************************************************************
 * Name: mq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task.  After a successful call of mq_open(), the task can
 *   reference the message queue using the address returned by the call. The
 *   message queue remains usable until it is closed by a successful call to
 *   mq_close().
 *
 * Input Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode (ignored), and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Returned Value:
 *   A message queue descriptor or (mqd_t)-1 (ERROR)
 *
 * Assumptions:
 *
 ****************************************************************************/

mqd_t mq_open(FAR const char *mq_name, int oflags, ...)
{
  FAR struct mq_attr *attr = NULL;
  mode_t mode = 0;
  mqd_t mqdes;
  va_list ap;
  int ret;

  /* Were we asked to create it? */

  if ((oflags & O_CREAT) != 0)
    {
      /* We have to extract the additional
       * parameters from the variable argument list.
       */

      va_start(ap, oflags);
      mode = va_arg(ap, mode_t);
      attr = va_arg(ap, FAR struct mq_attr *);
      va_end(ap);
    }

  ret = nxmq_open(mq_name, oflags, mode, attr, &mqdes);
  if (ret < 0)
    {
      set_errno(-ret);
      mqdes = (mqd_t)ERROR;
    }

  return mqdes;
}
