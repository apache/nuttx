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

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

#include "inode/inode.h"
#include "sched/sched.h"
#include "vfs.h"

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
#ifdef CONFIG_FS_NOTIFY
      else if (path != NULL)
        {
          lib_put_pathbuffer(path);
        }
#endif

      filep->f_inode = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   nx_close() is similar to the standard 'close' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_close() is an internal NuttX interface and should not be called from
 *   applications.
 *
 *   Close an inode (if open)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   on any failure.
 *
 * Assumptions:
 *   Caller holds the list mutex because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

int nx_close(int fd)
{
  return fdlist_close(nxsched_get_fdlist_from_tcb(this_task()), fd);
}

/****************************************************************************
 * Name: close
 *
 * Description:
 *   close() closes a file descriptor, so that it no longer refers to any
 *   file and may be reused. Any record locks (see fcntl(2)) held on the file
 *   it was associated with, and owned by the process, are removed
 *   (regardless of the file descriptor that was used to obtain the lock).
 *
 *   If fd is the last copy of a particular file descriptor the resources
 *   associated with it are freed; if the descriptor was the last reference
 *   to a file which has been removed using unlink(2) the file is deleted.
 *
 * Input Parameters:
 *   fd   file descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

int close(int fd)
{
  int ret;

#ifdef CONFIG_FDSAN
  android_fdsan_exchange_owner_tag(fd, 0, 0);
#endif

  /* close() is a cancellation point */

  enter_cancellation_point();

  ret = nx_close(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
