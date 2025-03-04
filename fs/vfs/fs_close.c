/****************************************************************************
 * fs/vfs/fs_close.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>

#include "notify/notify.h"
#include "inode/inode.h"
#include "vfs/lock.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_FS_NOTIFY
static FAR char *file_get_path(FAR struct file *filep)
{
  FAR char *pathbuffer;
  int ret;

  pathbuffer = lib_get_pathbuffer();
  if (pathbuffer == NULL)
    {
      return NULL;
    }

  ret = file_fcntl(filep, F_GETPATH, pathbuffer);
  if (ret < 0)
    {
      lib_put_pathbuffer(pathbuffer);
      return NULL;
    }

  return pathbuffer;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_close_without_clear
 *
 * Description:
 *   Close a file that was previously opened with file_open(), but without
 *   clear filep.
 *
 * Input Parameters:
 *   filep - A pointer to a user provided memory location containing the
 *           open file data returned by file_open().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_close_without_clear(FAR struct file *filep)
{
  struct inode *inode;
#ifdef CONFIG_FS_NOTIFY
  FAR char *path;
#endif
  int ret = OK;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

#ifdef CONFIG_FS_NOTIFY
  /* We lose the path and inode during close and release, so obtain it
   * in advance. Then we pass it to notify_close function.
   */

  path = file_get_path(filep);
#endif

  /* Check if the struct file is open (i.e., assigned an inode) */

  if (inode)
    {
      file_closelk(filep);

      /* Close the file, driver, or mountpoint. */

      if (inode->u.i_ops && inode->u.i_ops->close)
        {
          /* Perform the close operation */

          ret = inode->u.i_ops->close(filep);
        }

      /* And release the inode */

      if (ret >= 0)
        {
#ifdef CONFIG_FS_NOTIFY
          if (path != NULL)
            {
              notify_close(path, filep->f_oflags);
              lib_put_pathbuffer(path);
            }
#endif

          inode_release(inode);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: file_close
 *
 * Description:
 *   Close a file that was previously opened with file_open().
 *
 * Input Parameters:
 *   filep - A pointer to a user provided memory location containing the
 *           open file data returned by file_open().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_close(FAR struct file *filep)
{
  int ret;

  ret = file_close_without_clear(filep);
  if (ret >= 0 && filep->f_inode)
    {
      /* Reset the user file struct instance so that it cannot be reused. */

      filep->f_inode = NULL;

#ifdef CONFIG_FDCHECK
      filep->f_tag_fdcheck = 0;
#endif

#ifdef CONFIG_FDSAN
      filep->f_tag_fdsan = 0;
#endif
    }

  return ret;
}
