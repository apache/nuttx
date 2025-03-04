/****************************************************************************
 * fs/vfs/fs_syncfs.c
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
#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_syncfs
 *
 * Description:
 *   Equivalent to the standard syncfs() function except that is accepts a
 *   struct file instance instead of a fd descriptor and it does not set
 *   the errno variable
 *
 ****************************************************************************/

int file_syncfs(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;

  if (inode != NULL)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops &&
          inode->u.i_mops->syncfs)
        {
          return inode->u.i_mops->syncfs(inode);
        }
#endif /* !CONFIG_DISABLE_MOUNTPOINT */
    }

  return -EBADF;
}

/****************************************************************************
 * Name: syncfs
 *
 * Description:
 *   syncfs() is like sync(), but synchronizes just the filesystem
 *   containing file referred to by the open file descriptor fd.
 *
 * Returned Value:
 *   syncfs() returns 0 on success; on error, it returns -1 and sets
 *   errno to indicate the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int syncfs(int fd)
{
  FAR struct file *filep;
  int ret;

  enter_cancellation_point();

  ret = fs_getfilep(fd, &filep);
  if (ret == OK)
    {
      ret = file_syncfs(filep);
      fs_putfilep(filep);
    }

  leave_cancellation_point();

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

