/****************************************************************************
 * fs/vfs/fs_fstatfs.c
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

#include <sys/statfs.h>
#include <string.h>
#include <limits.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fstatfs
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set:
 *
 *   EACCES  Search permission is denied for one of the directories in the
 *           path prefix of path.
 *   EFAULT  Bad address.
 *   ENOENT  A component of the path path does not exist, or the path is an
 *           empty string.
 *   ENOMEM  Out of memory
 *   ENOTDIR A component of the path is not a directory.
 *   ENOSYS  The file system does not support this call.
 *
 ****************************************************************************/

int fstatfs(int fd, FAR struct statfs *buf)
{
  FAR struct file *filep;
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(buf != NULL);

  /* First, get the file structure.  Note that on failure,
   * fs_getfilep() will return the errno.
   */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Get the inode from the file structure */

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);

  /* Check if the file is open */

  if (inode == NULL)
    {
      /* The descriptor does not refer to an open file. */

      ret = -EBADF;
    }
  else
#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* The way we handle the stat depends on the type of inode that we
   * are dealing with.
   */

  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the statfs() method
       */

      ret = -ENOSYS;
      if (inode->u.i_mops && inode->u.i_mops->statfs)
        {
          /* Perform the statfs() operation */

          ret = inode->u.i_mops->statfs(inode, buf);
        }
    }
  else
#endif
    {
      /* The node is part of the root pseudo file system */

      memset(buf, 0, sizeof(struct statfs));
      buf->f_type    = PROC_SUPER_MAGIC;
      buf->f_namelen = NAME_MAX;
      ret            = OK;
    }

  /* Check if the fstat operation was successful */

  if (ret >= 0)
    {
      /* Successfully statfs'ed the file */

      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}
