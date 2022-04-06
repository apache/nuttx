/****************************************************************************
 * fs/vfs/fs_truncate.c
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
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "inode/inode.h"

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_truncate
 *
 * Description:
 *   Equivalent to the standard ftruncate() function except that is accepts
 *   a struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_truncate(FAR struct file *filep, off_t length)
{
  struct inode *inode;

  /* Was this file opened for write access? */

  if ((filep->f_oflags & O_WROK) == 0)
    {
      fwarn("WARNING: Cannot truncate a file opened read-only\n");
      return -EBADF;
    }

  /* Is this inode a registered mountpoint? Does it support the
   * truncate operations may be relevant to device drivers but only
   * the mountpoint operations vtable contains a truncate method.
   */

  inode = filep->f_inode;
  if (inode == NULL)
    {
      return -EINVAL;
    }

  /* If inode is not mountpoint try ioctl first */

  if (!INODE_IS_MOUNTPT(inode))
    {
      return file_ioctl(filep, FIOC_TRUNCATE, length);
    }

  if (inode->u.i_mops == NULL)
    {
      fwarn("WARNING:  Not a (regular) file on a mounted file system.\n");
      return -EINVAL;
    }

  /* A NULL write() method is an indicator of a read-only file system (but
   * possible not the only indicator -- sufficient, but not necessary")
   */

  if (inode->u.i_mops->write == NULL)
    {
      fwarn("WARNING: File system is read-only\n");
      return -EROFS;
    }

  /* Does the file system support the truncate method?  It should if it is
   * a write-able file system.
   */

  if (inode->u.i_mops->truncate == NULL)
    {
      fwarn("WARNING: File system does not support the truncate() method\n");
      return -ENOSYS;
    }

  /* Yes, then tell the file system to truncate this file */

  return inode->u.i_mops->truncate(filep, length);
}

/****************************************************************************
 * Name: ftruncate
 *
 * Description:
 *   The ftruncate() function causes the regular file referenced by fd to
 *   have a size of length bytes.
 *
 *   If the file previously was larger than length, the extra data is
 *   discarded.  If it was previously shorter than length, it is unspecified
 *   whether the file is changed or its size increased.  If the file is
 *   extended, the extended area appears as if it were zero-filled.  If fd
 *   references a shared memory object, ftruncate() sets the size of the
 *   shared memory object to length. If the file is not a regular file or
 *   a shared memory object, the result is unspecified.

 *   With ftruncate(), the file must be open for writing; for truncate(),
 *   the process must have write permission for the file.
 *
 *   ftruncate() does not modify the file offset for any open file
 *   descriptions associated with the file.
 *
 * Input Parameters:
 *   fd     - A reference to an open, regular file or shared memory object
 *            to be truncated.
 *   length - The new length of the file or shared memory object.
 *
 * Returned Value:
 *    Upon successful completion, ftruncate() return 0s. Otherwise a -1 is
 *    returned, and errno is set to indicate the error.
 *
 *    EINTR
 *      - A signal was caught during execution.
 *    EINVAL
 *      - The length argument was less than 0.
 *    EFBIG or EINVAL
 *      - The length argument was greater than the maximum file size.
 *    EIO
 *      - An I/O error occurred while reading from or writing to a file
 *        system.
 *    EBADF or EINVAL
 *       - the fd argument is not a file descriptor open for writing.
 *    EFBIG
 *       - The file is a regular file and length is greater than the offset
 *         maximum established in the open file description associated with
 *         fd.
 *    EINVAL
 *       - The fd argument references a file that was opened without write
 *         permission.
 *    EROFS
 *       - The named file resides on a read-only file system.
 *
 ****************************************************************************/

int ftruncate(int fd, off_t length)
{
  FAR struct file *filep;
  int ret = -EINVAL;

  if (length < 0)
    {
      goto errout;
    }

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      ferr("ERROR: Could no get file structure: %d\n", ret);
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Perform the truncate operation */

  ret = file_truncate(filep, length);
  if (ret >= 0)
    {
      return 0;
    }

  fwarn("WARNING: file_truncate() failed: %d\n", ret);

errout:
  set_errno(-ret);
  return ERROR;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT */
