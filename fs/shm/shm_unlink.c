/****************************************************************************
 * fs/shm/shm_unlink.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int file_shm_unlink(FAR const char *name)
{
  FAR struct inode *inode;
  struct inode_search_s desc;
  char fullpath[sizeof(CONFIG_FS_SHMFS_VFS_PATH) + CONFIG_NAME_MAX + 2];
  int ret;

  /* Make sure that a non-NULL name is supplied */

  if (!name)
    {
      return -ENOENT;
    }

  /* Remove any number of leading '/' */

  while (*name == '/')
    {
      name++;
    }

  /* Empty name supplied? */

  if (*name == '\0')
    {
      return -ENOENT;
    }

  /* Name too long? */

  if (strnlen(name, CONFIG_NAME_MAX + 1) > CONFIG_NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  /* Get the full path to the shm object */

  snprintf(fullpath, sizeof(fullpath),
           CONFIG_FS_SHMFS_VFS_PATH "/%s", name);

  /* Get the inode for this shm object */

  SETUP_SEARCH(&desc, fullpath, false);

  ret = inode_lock();
  if (ret < 0)
    {
      goto errout_with_search;
    }

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      goto errout_with_sem;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that what we found is, indeed, an shm inode */

  if (!INODE_IS_SHM(inode))
    {
      ret = -ENOENT;
      goto errout_with_inode;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (inode->u.i_ops->unlink)
    {
      /* Notify the shmfs driver that it has been unlinked */

      ret = inode->u.i_ops->unlink(inode);
      if (ret < 0)
        {
          goto errout_with_inode;
        }
    }
#endif

  /* Remove the old inode from the tree. If we hold a reference count
   * on the inode, it will not be deleted now.  This will set the
   * FSNODEFLAG_DELETED bit in the inode flags.
   */

  ret = inode_remove(fullpath);

  /* inode_remove() should always fail with -EBUSY because we have a
   * reference on the inode.  -EBUSY means that the inode was, indeed,
   * unlinked but it could not be freed because there are references.
   */

  if (ret == -EBUSY)
    {
      ret = OK;
    }

  DEBUGASSERT(ret == OK);

errout_with_inode:
  inode_release(inode);
errout_with_sem:
  inode_unlock();
errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int shm_unlink(FAR const char *name)
{
  int ret = file_shm_unlink(name);

  if (ret < 0)
    {
      set_errno(-ret);
      return -1;
    }

  return 0;
}
