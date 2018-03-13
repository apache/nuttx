/****************************************************************************
 * fs/vfs/fs_pread.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_pread
 *
 * Description:
 *   Equivalent to the standard pread function except that is accepts a
 *   struct file instance instead of a file descriptor.  Currently used
 *   only by aio_read();
 *
 ****************************************************************************/

ssize_t file_pread(FAR struct file *filep, FAR void *buf, size_t nbytes,
                   off_t offset)
{
  off_t savepos;
  off_t pos;
  ssize_t ret;

  /* Perform the seek to the current position.  This will not move the
   * file pointer, but will return its current setting
   */

  savepos = file_seek(filep, 0, SEEK_CUR);
  if (savepos < 0)
    {
      /* file_seek might fail if this if the media is not seekable */

      return (ssize_t)savepos;
    }

  /* Then seek to the correct position in the file */

  pos = file_seek(filep, offset, SEEK_SET);
  if (pos < 0)
    {
      /* This might fail is the offset is beyond the end of file */

      return (ssize_t)pos;
    }

  /* Then perform the read operation */

  ret = file_read(filep, buf, nbytes);

  /* Restore the file position */

  pos = file_seek(filep, savepos, SEEK_SET);
  if (pos < 0 && ret >= 0)
    {
      /* This really should not fail */

      ret = (ssize_t)pos;
    }

  return ret;
}

/****************************************************************************
 * Name: pread
 *
 * Description:
 *   The pread() function performs the same action as read(), except that it
 *   reads from a given position in the file without changing the file
 *   pointer. The first three arguments to pread() are the same as read()
 *   with the addition of a fourth argument offset for the desired position
 *   inside the file. An attempt to perform a pread() on a file that is
 *   incapable of seeking results in an error.
 *
 *   NOTE: This function could have been wholly implemented within libc but
 *   it is not.  Why?  Because if pread were implemented in libc, it would
 *   require four system calls.  If it is implemented within the kernel,
 *   only three.
 *
 * Input Parameters:
 *   file     File structure instance
 *   buf      User-provided to save the data
 *   nbytes   The maximum size of the user-provided buffer
 *   offset   The file offset
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or -1 on failure with errno set appropriately.
 *   See read() return values
 *
 ****************************************************************************/

ssize_t pread(int fd, FAR void *buf, size_t nbytes, off_t offset)
{
  FAR struct file *filep;
  ssize_t ret;

  /* pread() is a cancellation point */

  (void)enter_cancellation_point();

  /* Get the file structure corresponding to the file descriptor. */

  ret = (ssize_t)fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  DEBUGASSERT(filep != NULL);

  /* Let file_pread do the real work */

  ret = file_pread(filep, buf, nbytes, offset);
  if (ret < 0)
    {
      goto errout;
    }

  leave_cancellation_point();
  return ret;

errout:
  set_errno((int)-ret);
  leave_cancellation_point();
  return (ssize_t)ERROR;
}
