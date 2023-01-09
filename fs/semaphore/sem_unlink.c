/****************************************************************************
 * fs/semaphore/sem_unlink.c
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
#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "inode/inode.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_unlink
 *
 * Description:
 *   This function removes the semaphore named by the input parameter 'name.'
 *   If the semaphore named by 'name' is currently referenced by other task,
 *   the sem_unlink() will have no effect on the state of the semaphore.  If
 *   one or more processes have the semaphore open when sem_unlink() is
 *   called, destruction of the semaphore will be postponed until all
 *   references to the semaphore have been destroyed by calls of sem_close().
 *
 * Input Parameters:
 *   name - Semaphore name
 *
 * Returned Value:
 *  0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sem_unlink(FAR const char *name)
{
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[MAX_SEMPATH];
  int errcode;
  int ret;

  /* Get the full path to the semaphore */

  snprintf(fullpath, MAX_SEMPATH,
           CONFIG_FS_NAMED_SEMAPHORES_VFS_PATH "/%s", name);

  /* Get the inode for this semaphore. */

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

  /* Verify that what we found is, indeed, a semaphore */

  if (!INODE_IS_NAMEDSEM(inode))
    {
      errcode = ENOENT;
      goto errout_with_inode;
    }

  /* Refuse to unlink the inode if it has children.  I.e., if it is
   * functioning as a directory and the directory is not empty.
   */

  ret = inode_lock();
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_inode;
    }

  if (inode->i_child != NULL)
    {
      errcode = ENOTEMPTY;
      goto errout_with_lock;
    }

  /* Remove the old inode from the tree.  Because we hold a reference count
   * on the inode, it will not be deleted now.  This will set the
   * FSNODEFLAG_DELETED bit in the inode flags.
   */

  ret = inode_remove(fullpath);

  /* inode_remove() should always fail with -EBUSY because we hae a reference
   * on the inode.  -EBUSY means that the inode was, indeed, unlinked but
   * thatis could not be freed because there are references.
   */

  DEBUGASSERT(ret >= 0 || ret == -EBUSY);
  UNUSED(ret);

  /* Now we do not release the reference count in the normal way (by calling
   * inode release.  Rather, we call sem_close().  sem_close will decrement
   * the reference count on the inode.  But it will also free the semaphore
   * if that reference count decrements to zero.  Since we hold one
   * reference, that can only occur if the semaphore is not in-use.
   */

  inode_unlock();
  ret = sem_close(&inode->u.i_nsem->ns_sem);
  RELEASE_SEARCH(&desc);
  sched_unlock();
  return ret;

errout_with_lock:
  inode_unlock();

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  sched_unlock();
  return ERROR;
}
