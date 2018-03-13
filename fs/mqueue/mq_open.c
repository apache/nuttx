/****************************************************************************
 *  fs/mqueue/mq_open.c
 *
 *   Copyright (C) 2007-2009, 2011, 2014-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  FAR struct inode *inode;
  FAR struct mqueue_inode_s *msgq;
  struct inode_search_s desc;
  char fullpath[MAX_MQUEUE_PATH];
  va_list ap;
  struct mq_attr *attr;
  mqd_t mqdes;
  mode_t mode;
  int errcode;
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (mq_name == NULL || *mq_name == '\0')
    {
      errcode = EINVAL;
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
          errcode = ENXIO;
          goto errout_with_inode;
        }

      /* It exists and is a message queue.  Check if the caller wanted to
       * create a new mqueue with this name.
       */

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          errcode = EEXIST;
          goto errout_with_inode;
        }

      /* Create a message queue descriptor for the current thread */

      msgq  = inode->u.i_mqueue;
      mqdes = nxmq_create_des(NULL, msgq, oflags);
      if (!mqdes)
        {
          errcode = ENOMEM;
          goto errout_with_inode;
        }
    }
  else
    {
      /* The mqueue does not exists.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* The mqueue does not exist and O_CREAT is not set */

          errcode = ENOENT;
          goto errout_with_lock;
        }

      /* Create the mqueue.  First we have to extract the additional
       * parameters from the variable argument list.
       */

      va_start(ap, oflags);
      mode = va_arg(ap, mode_t);
      attr = va_arg(ap, FAR struct mq_attr *);
      va_end(ap);

      /* Create an inode in the pseudo-filesystem at this path */

      inode_semtake();
      ret = inode_reserve(fullpath, &inode);
      inode_semgive();

      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_lock;
        }

      /* Allocate memory for the new message queue.  The new inode will
       * be created with a reference count of zero.
       */

      msgq = (FAR struct mqueue_inode_s *)nxmq_alloc_msgq(mode, attr);
      if (!msgq)
        {
          errcode = ENOSPC;
          goto errout_with_inode;
        }

      /* Create a message queue descriptor for the TCB */

       mqdes = nxmq_create_des(NULL, msgq, oflags);
       if (!mqdes)
         {
           errcode = ENOMEM;
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
  return mqdes;

errout_with_msgq:
  nxmq_free_msgq(msgq);
  inode->u.i_mqueue = NULL;

errout_with_inode:
  inode_release(inode);

errout_with_lock:
  RELEASE_SEARCH(&desc);
  sched_unlock();

errout:
  set_errno(errcode);
  return (mqd_t)ERROR;
}
