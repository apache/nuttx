/****************************************************************************
 * fs/vfs/fs_fsync.c
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
#include <fcntl.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_fsync
 *
 * Description:
 *   Equivalent to the standard fsync() function except that is accepts a
 *   struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_fsync(FAR struct file *filep)
{
  FAR struct inode *inode;
  int ret;

  /* Is this inode a registered mountpoint? Does it support the
   * sync operations may be relevant to device drivers but only
   * the mountpoint operations vtable contains a sync method.
   */

  inode = filep->f_inode;
  if (inode != NULL)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          if (inode->u.i_mops && inode->u.i_mops->sync)
            {
              /* Yes, then tell the mountpoint to sync this file */

              return inode->u.i_mops->sync(filep);
            }
        }
      else
#endif
      if (inode->u.i_ops && inode->u.i_ops->ioctl)
        {
          ret = inode->u.i_ops->ioctl(filep, BIOC_FLUSH, 0);
          return ret >= 0 ? 0 : ret;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: fsync
 *
 * Description:
 *   This func simply binds inode sync methods to the sync system call.
 *
 ****************************************************************************/

int fsync(int fd)
{
  FAR struct file *filep;
  int ret;

  /* fsync() is a cancellation point */

  enter_cancellation_point();

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Perform the fsync operation */

  ret = file_fsync(filep);
  if (ret < 0)
    {
      goto errout;
    }

  leave_cancellation_point();
  return ret;

errout:
  leave_cancellation_point();
  set_errno(-ret);
  return ERROR;
}
