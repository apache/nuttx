/****************************************************************************
 * fs/mqueue/mq_unlink.c
 *
 *   Copyright (C) 2007, 2009, 2014, 2017 Gregory Nutt. All rights reserved.
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
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[MAX_MQUEUE_PATH];
  int errcode;
  int ret;

  /* Get the full path to the message queue */

  snprintf(fullpath, MAX_MQUEUE_PATH, CONFIG_FS_MQUEUE_MPATH "/%s", mq_name);

  /* Get the inode for this message queue. */

  SETUP_SEARCH(&desc, fullpath, false);

  sched_lock();
  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      errcode = -ret;
      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that what we found is, indeed, a message queue */

  if (!INODE_IS_MQUEUE(inode))
    {
      errcode = ENXIO;
      goto errout_with_inode;
    }

  /* Refuse to unlink the inode if it has children.  I.e., if it is
   * functioning as a directory and the directory is not empty.
   */

  inode_semtake();
  if (inode->i_child != NULL)
    {
      errcode = ENOTEMPTY;
      goto errout_with_semaphore;
    }

  /* Remove the old inode from the tree.  Because we hold a reference count
   * on the inode, it will not be deleted now.  This will set the
   * FSNODEFLAG_DELETED bit in the inode flags.
   */

  ret = inode_remove(fullpath);

  /* inode_remove() should always fail with -EBUSY because we hae a reference
   * on the inode.  -EBUSY means taht the inode was, indeed, unlinked but
   * thatis could not be freed because there are refrences.
   */

  DEBUGASSERT(ret >= 0 || ret == -EBUSY);
  UNUSED(ret);

  /* Now we do not release the reference count in the normal way (by calling
   * inode release.  Rather, we call mq_inode_release().  mq_inode_release
   * will decrement the reference count on the inode.  But it will also free
   * the message queue if that reference count decrements to zero.  Since we
   * hold one reference, that can only occur if the message queue is not
   * in-use.
   */

  inode_semgive();
  mq_inode_release(inode);
  RELEASE_SEARCH(&desc);
  sched_unlock();
  return OK;

errout_with_semaphore:
  inode_semgive();

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  sched_unlock();
  return ERROR;
}
