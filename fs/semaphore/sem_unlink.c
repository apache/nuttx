/****************************************************************************
 * fs/semaphore/sem_unlink.c
 *
 *   Copyright (C) 2007-2009, 2014, 2017 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <sched.h>
#include <queue.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/kmalloc.h>

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

  snprintf(fullpath, MAX_SEMPATH, CONFIG_FS_NAMED_SEMPATH "/%s", name);

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
   * inode release.  Rather, we call sem_close().  sem_close will decrement
   * the reference count on the inode.  But it will also free the semaphore
   * if that reference count decrements to zero.  Since we hold one reference,
   * that can only occur if the semaphore is not in-use.
   */

  inode_semgive();
  ret = sem_close((FAR sem_t *)inode->u.i_nsem);
  RELEASE_SEARCH(&desc);
  sched_unlock();
  return ret;

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
