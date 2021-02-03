/****************************************************************************
 * fs/vfs/fs_close.c
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
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_NET
# include <nuttx/net/net.h>
#endif

#include "inode/inode.h"

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
  int ret = OK;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  /* Check if the struct file is open (i.e., assigned an inode) */

  if (inode)
    {
      /* Close the file, driver, or mountpoint. */

      if (inode->u.i_ops && inode->u.i_ops->close)
        {
          /* Perform the close operation */

          ret = inode->u.i_ops->close(filep);
        }

      /* And release the inode */

      inode_release(inode);

      /* Reset the user file struct instance so that it cannot be reused. */

      filep->f_oflags = 0;
      filep->f_pos    = 0;
      filep->f_inode  = NULL;
      filep->f_priv   = NULL;
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
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_close(int fd)
{
  /* Did we get a valid file descriptor? */

  if (fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Close a socket descriptor */

#ifdef CONFIG_NET
      if (fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS))
        {
          return net_close(fd);
        }
      else
#endif
        {
          return -EBADF;
        }
    }

  /* Close the driver or mountpoint.  NOTES: (1) there is no
   * exclusion mechanism here, the driver or mountpoint must be
   * able to handle concurrent operations internally, (2) The driver
   * may have been opened numerous times (for different file
   * descriptors) and must also handle being closed numerous times.
   * (3) for the case of the mountpoint, we depend on the close
   * methods bing identical in signature and position in the operations
   * vtable.
   */

  return files_close(fd);
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
