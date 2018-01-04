/****************************************************************************
 * fs/vfs/fs_truncate.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/fs/fs.h>

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
  if (inode == NULL || !INODE_IS_MOUNTPT(inode) || inode->u.i_mops == NULL)
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
  int ret;

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
