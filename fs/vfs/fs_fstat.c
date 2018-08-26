/****************************************************************************
 * fs/vfs/fs_fstat.c
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_fstat
 *
 * Description:
 *   file_fstat() is an internal OS interface.  It is functionally similar to
 *   the standard fstat() interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - The caller provide location in which to return information about
 *            the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall be
 *   returned and errno set to indicate the error.
 *
 ****************************************************************************/

int file_fstat(FAR struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  int ret;

  DEBUGASSERT(filep != NULL);

  /* Get the inode from the file structure */

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);

  /* The way we handle the stat depends on the type of inode that we
   * are dealing with.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (INODE_IS_MOUNTPT(inode))
    {
      /* The inode is a file system mountpoint. Verify that the mountpoint
       * supports the fstat() method
       */

      ret = -ENOSYS;
      if (inode->u.i_mops && inode->u.i_mops->fstat)
        {
          /* Perform the fstat() operation */

          ret = inode->u.i_mops->fstat(filep, buf);
        }
    }
  else
#endif
    {
      /* The inode is part of the root pseudo file system. */

      ret = inode_stat(inode, buf);
    }

  return ret;
}

/****************************************************************************
 * Name: fstat
 *
 * Description:
 *   The fstat() function will obtain information about an open file
 *   associated with the file descriptor 'fd', and will write it to the area
 *   pointed to by 'buf'.
 *
 *   The 'buf' argument is a pointer to a stat structure, as defined in
 *   <sys/stat.h>, into which information is placed concerning the file.
 *
 * Input Parameters:
 *   fd  - The file descriptor associated with the open file of interest
 *   buf - The caller provide location in which to return information about
 *         the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall be
 *   returned and errno set to indicate the error.
 *
 ****************************************************************************/

int fstat(int fd, FAR struct stat *buf)
{
  FAR struct file *filep;
  int ret;

  /* Did we get a valid file descriptor? */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
#if CONFIG_NSOCKET_DESCRIPTORS > 0
      /* Let the networking logic handle the fstat() */

      ret = net_fstat(fd, buf);
      if (ret < 0)
        {
          goto errout;
        }

      return OK;
#else
      /* No networking... it is just a bad descriptor */

      ret = -EBADF;
      goto errout;
#endif
    }

  /* The descriptor is in a valid range for a file descriptor... do the
   * fstat.  First, get the file structure.  Note that on failure,
   * fs_getfilep() will set the errno variable.
   */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  /* Perform the fstat operation */

  ret = file_fstat(filep, buf);

  /* Check if the fstat operation was successful */

  if (ret >= 0)
    {
      /* Successfully fstat'ed the file */

      return OK;
    }

errout:
  set_errno(-ret);
  return ERROR;
}
