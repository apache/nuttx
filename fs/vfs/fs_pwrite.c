/****************************************************************************
 * fs/vfs/fs_pwrite.c
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

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_pwrite
 *
 * Description:
 *   Equivalent to the standard pwrite function except that is accepts a
 *   struct file instance instead of a file descriptor.  Currently used
 *   only by aio_write();
 *
 ****************************************************************************/

ssize_t file_pwrite(FAR struct file *filep, FAR const void *buf,
                    size_t nbytes, off_t offset)
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

  /* Then perform the write operation */

  ret = file_write(filep, buf, nbytes);

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
 * Name: pwrite
 *
 * Description:
 *   The pwrite() function performs the same action as write(), except that
 *   it writes into a given position without changing the file pointer. The
 *   first three arguments to pwrite() are the same as write() with the
 *   addition of a fourth argument offset for the desired position inside
 *   the file.
 *
 *   NOTE: This function could have been wholly implemented within libc but
 *   it is not.  Why?  Because if pwrite were implemented in libc, it would
 *   require four system calls.  If it is implemented within the kernel,
 *   only three.
 *
 * Input Parameters:
 *   fd       file descriptor (or socket descriptor) to write to
 *   buf      Data to write
 *   nbytes   Length of data to write
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or -1 on failure with errno set appropriately.
 *   See write() return values
 *
 * Assumptions/Limitations:
 *   POSIX requires that opening a file with the O_APPEND flag should have no
 *   effect on the location at which pwrite() writes data.  However, on NuttX
 *   like on Linux, if a file is opened with O_APPEND, pwrite() appends data
 *   to the end of the file, regardless of the value of offset.
 *
 ****************************************************************************/

ssize_t pwrite(int fd, FAR const void *buf, size_t nbytes, off_t offset)
{
  FAR struct file *filep;
  ssize_t ret;

  /* pread() is a cancellation point */

  enter_cancellation_point();

  /* Get the file structure corresponding to the file descriptor. */

  ret = (ssize_t)fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  /* Let file_pwrite do the real work */

  ret = file_pwrite(filep, buf, nbytes, offset);
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
