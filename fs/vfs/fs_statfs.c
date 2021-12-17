/****************************************************************************
 * fs/vfs/fs_statfs.c
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: statpseudo
 ****************************************************************************/

static int statpseudofs(FAR struct inode *inode, FAR struct statfs *buf)
{
  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = PROC_SUPER_MAGIC;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: statfs
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

int statfs(FAR const char *path, FAR struct statfs *buf)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret = OK;

  /* Sanity checks */

  if (path == NULL  || buf == NULL)
    {
      ret = -EFAULT;
      goto errout;
    }

  if (*path == '\0')
    {
      ret = -ENOENT;
      goto errout;
    }

  /* Get an inode for this file */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* This name does not refer to a psudeo-inode and there is no
       * mountpoint that includes in this path.
       */

      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* The way we handle the statfs depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The node is a file system mointpoint. Verify that the mountpoint
       * supports the statfs() method
       */

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

      ret = statpseudofs(inode, buf);
    }

  /* Check if the statfs operation was successful */

  if (ret < 0)
    {
      goto errout_with_inode;
    }

  /* Successfully statfs'ed the file */

  inode_release(inode);
  RELEASE_SEARCH(&desc);
  return OK;

  /* Failure conditions always set the errno appropriately */

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);

errout:
  set_errno(-ret);
  return ERROR;
}
