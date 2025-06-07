/****************************************************************************
 * fs/vfs/fs_dup2.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include "inode/inode.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup2
 *
 * Description:
 *   Assign an inode to a specific files structure.  This is the heart of
 *   dup2.
 *
 *   Equivalent to the non-standard dup2() function except that it
 *   accepts struct file instances instead of file descriptors and it does
 *   not set the errno variable.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int file_dup2(FAR struct file *filep1, FAR struct file *filep2)
{
  FAR struct inode *inode;
  int ret;

  if (filep1 == NULL || filep1->f_inode == NULL || filep2 == NULL)
    {
      return -EBADF;
    }

  if (filep1 == filep2)
    {
      return OK;
    }

  /* Increment the reference count on the contained inode */

  inode = filep1->f_inode;
  inode_addref(inode);

  /* Close the second file */

  ret = file_close(filep2);
  if (ret < 0)
    {
      inode_release(inode);
      return ret;
    }

  filep2->f_priv  = NULL;
  filep2->f_pos   = filep1->f_pos;
  filep2->f_inode = inode;

  /* Call the open method on the file, driver, mountpoint so that it
   * can maintain the correct open counts.
   */

  if (inode->u.i_ops)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          /* Dup the open file on the in the new file structure */

          if (inode->u.i_mops->dup)
            {
              ret = inode->u.i_mops->dup(filep1, filep2);
            }
        }
      else
#endif
        {
          /* (Re-)open the pseudo file or device driver */

          filep2->f_priv = filep1->f_priv;

          /* Add nonblock flags to avoid happening block when
           * calling open()
           */

          filep2->f_oflags |= O_NONBLOCK;

          if (inode->u.i_ops->open)
            {
              ret = inode->u.i_ops->open(filep2);
            }

          if (ret >= 0 && (filep1->f_oflags & O_NONBLOCK) == 0)
            {
              ret = file_ioctl(filep2, FIONBIO, 0);
              if (ret < 0 && inode->u.i_ops->close)
                {
                  inode->u.i_ops->close(filep2);
                }
            }
        }

      /* Handle open failures */

      if (ret < 0)
        {
          inode_release(inode);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: nx_dup2
 *
 * Description:
 *   nx_dup2() is similar to the standard 'dup2' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup2() is an internal NuttX interface and should not be called from
 *   applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2(int fd1, int fd2)
{
  return fdlist_dup2(nxsched_get_fdlist_from_tcb(this_task()), fd1, fd2);
}

/****************************************************************************
 * Name: dup2
 *
 * Description:
 *   Clone a file descriptor or socket descriptor to a specific descriptor
 *   number
 *
 ****************************************************************************/

int dup2(int fd1, int fd2)
{
  int ret;

  ret = nx_dup2(fd1, fd2);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
