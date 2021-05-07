/****************************************************************************
 * fs/vfs/fs_write.c
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
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/cancelpt.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_write
 *
 * Description:
 *   Equivalent to the standard write() function except that is accepts a
 *   struct file instance instead of a file descriptor.  It is functionally
 *   equivalent to write() except that in addition to the differences in
 *   input parameters:
 *
 *  - It does not modify the errno variable,
 *  - It is not a cancellation point, and
 *
 * Input Parameters:
 *   filep  - Instance of struct file to use with the write
 *   buf    - Data to write
 *   nbytes - Length of data to write
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments withwrite() for a description of the appropriate errno
 *  values).
 *
 ****************************************************************************/

ssize_t file_write(FAR struct file *filep, FAR const void *buf,
                   size_t nbytes)
{
  FAR struct inode *inode;

  /* Was this file opened for write access? */

  if ((filep->f_oflags & O_WROK) == 0)
    {
      return -EACCES;
    }

  /* Is a driver registered? Does it support the write method? */

  inode = filep->f_inode;
  if (!inode || !inode->u.i_ops || !inode->u.i_ops->write)
    {
      return -EBADF;
    }

  /* Yes, then let the driver perform the write */

  return inode->u.i_ops->write(filep, buf, nbytes);
}

/****************************************************************************
 * Name: nx_write
 *
 * Description:
 *  nx_write() writes up to nytes bytes to the file referenced by the file
 *  descriptor fd from the buffer starting at buf.  nx_write() is an
 *  internal OS function.  It is functionally equivalent to write() except
 *  that:
 *
 *  - It does not modify the errno variable, and
 *  - It is not a cancellation point.
 *
 * Input Parameters:
 *   fd     - file descriptor to write to
 *   buf    - Data to write
 *   nbytes - Length of data to write
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_write(int fd, FAR const void *buf, size_t nbytes)
{
  FAR struct file *filep;
  ssize_t ret;

  if (buf == NULL)
    {
      return -EINVAL;
    }

  /* First, get the file structure.
   * Note that fs_getfilep() will return the errno on failure.
   */

  ret = (ssize_t)fs_getfilep(fd, &filep);
  if (ret >= 0)
    {
      /* Perform the write operation using the file descriptor as an
       * index.  Note that file_write() will return the errno on failure.
       */

      ret = file_write(filep, buf, nbytes);
    }

  return ret;
}

/****************************************************************************
 * Name: write
 *
 * Description:
 *  write() writes up to nytes bytes to the file referenced by the file
 *  descriptor fd from the buffer starting at buf.
 *
 * Input Parameters:
 *   fd     - file descriptor to write to
 *   buf    - Data to write
 *   nbytes - Length of data to write
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written). On error, -1 is returned, and errno is set appro-
 *  priately:
 *
 *  EAGAIN
 *    Non-blocking I/O has been selected using O_NONBLOCK and the write
 *    would block.
 *  EBADF
 *    fd is not a valid file descriptor or is not open for writing.
 *  EFAULT
 *    buf is outside your accessible address space.
 *  EFBIG
 *    An attempt was made to write a file that exceeds the implementation
 *    defined maximum file size or the process's file size limit, or
 *    to write at a position past the maximum allowed offset.
 *  EINTR
 *    The call was interrupted by a signal before any data was written.
 *  EINVAL
 *    fd is attached to an object which is unsuitable for writing; or
 *    the file was opened with the O_DIRECT flag, and either the address
 *    specified in buf, the value specified in count, or the current
 *     file offset is not suitably aligned.
 *  EIO
 *    A low-level I/O error occurred while modifying the inode.
 *  ENOSPC
 *    The device containing the file referred to by fd has no room for
 *    the data.
 *  EPIPE
 *    fd is connected to a pipe or socket whose reading end is closed.
 *    When this happens the writing process will also receive a SIGPIPE
 *    signal. (Thus, the write return value is seen only if the program
 *    catches, blocks or ignores this signal.)
 *
 ****************************************************************************/

ssize_t write(int fd, FAR const void *buf, size_t nbytes)
{
  ssize_t ret;

  /* write() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_write() do all of the work */

  ret = nx_write(fd, buf, nbytes);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
