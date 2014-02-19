/****************************************************************************
 * fs/fs_rmdir.c
 *
 *   Copyright (C) 2007-2009, 2014 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmdir
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int rmdir(FAR const char *pathname)
{
  FAR struct inode *inode;
  const char       *relpath = NULL;
  int               errcode;

  /* Get an inode for this file.  inode_find() automatically increments the
   * reference count on the inode if one is found.
   */

  inode = inode_find(pathname, &relpath);
  if (!inode)
    {
      /* There is no inode that includes in this path */

      errcode = ENOENT;
      goto errout;
    }

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* Check if the inode is a valid mountpoint. */

  if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops)
    {
      /* Perform the rmdir operation using the relative path
       * from the mountpoint.
       */

      if (inode->u.i_mops->rmdir)
        {
          int ret = inode->u.i_mops->rmdir(inode, relpath);
          if (ret < 0)
            {
              errcode = -ret;
              goto errout_with_inode;
            }
        }
      else
        {
          errcode = ENOSYS;
          goto errout_with_inode;
        }
    }
  else
#endif

  /* If this is a "dangling" pseudo-file node (i.e., it has no operations)
   * then rmdir should remove the node.
   */

  if (!inode->u.i_ops)
    {
      int ret;

      /* If the directory inode has children, however, then it cannot be
       * removed.
       */

      if (inode->i_child)
        {
          errcode = ENOTEMPTY;
          goto errout_with_inode;
        }

      /* Remove the inode.  NOTE: Because we hold a reference count on the
       * inode, it will not be deleted now.  But probably when inode_release()
       * is called below.  inode_remove should return -EBUSY to indicate that
       * the inode was not deleted now.
       */

      inode_semtake();
      ret = inode_remove(pathname);
      inode_semgive();

      if (ret < 0 && ret != -EBUSY)
        {
          errcode = -ret;
          goto errout_with_inode;
        }
    }
  else
    {
      errcode = ENXIO;
      goto errout_with_inode;
    }

  /* Successfully removed the directory */

  inode_release(inode);
  return OK;

errout_with_inode:
  inode_release(inode);
errout:
  set_errno(errcode);
  return ERROR;
}

