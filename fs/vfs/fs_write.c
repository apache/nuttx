/****************************************************************************
 * fs/vfs/fs_write.c
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

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/cancelpt.h>

#include "inode/inode.h"
#include "vfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_writev_compat
 *
 * Description:
 *   Emulate writev using file_operation::write.
 *
 *   Please read the comment on file_readv_compat as well.
 *
 ****************************************************************************/

static ssize_t file_writev_compat(FAR struct file *filep,
                                  FAR const struct iovec *iov, int iovcnt)
{
  FAR struct inode *inode = filep->f_inode;
  ssize_t nwritten;
  ssize_t ntotal;
  int i;

  /* Process each entry in the struct iovec array */

  for (i = 0, ntotal = 0; i < iovcnt; i++)
    {
      /* Ignore zero-length writes */

      if (iov[i].iov_len == 0)
        {
          continue;
        }

      /* Sanity check to avoid total length overflow */

      if (SSIZE_MAX - ntotal < iov[i].iov_len)
        {
          if (ntotal > 0)
            {
              break;
            }

          return -EINVAL;
        }

      nwritten = inode->u.i_ops->write(filep, iov[i].iov_base,
                                       iov[i].iov_len);

      /* Check for a write error */

      if (nwritten < 0)
        {
          if (ntotal > 0)
            {
              break;
            }

          return nwritten;
        }

      ntotal += nwritten;

      /* Check for a partial success condition */

      if (nwritten < iov[i].iov_len)
        {
          break;
        }
    }

  return ntotal;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_writev
 *
 * Description:
 *   Equivalent to the standard writev() function except that is accepts a
 *   struct file instance instead of a file descriptor.  It is functionally
 *   equivalent to writev() except that in addition to the differences in
 *   input parameters:
 *
 *  - It does not modify the errno variable,
 *  - It is not a cancellation point, and
 *
 * Input Parameters:
 *   filep  - Instance of struct file to use with the write
 *   iov    - Data to write
 *   iovcnt - The number of vectors
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments withwrite() for a description of the appropriate errno
 *  values).
 *
 ****************************************************************************/

ssize_t file_writev(FAR struct file *filep,
                    FAR const struct iovec *iov, int iovcnt)
{
  FAR struct inode *inode;
  ssize_t ret;

  /* Was this file opened for write access? */

  if ((filep->f_oflags & O_WROK) == 0)
    {
      return -EACCES;
    }

  /* Check buffer count and pointer for iovec */

  if (iovcnt == 0)
    {
      return 0;
    }

  if (iov == NULL)
    {
      return -EFAULT;
    }

  /* Are all iov_base accessible? */

  for (ret = 0; ret < iovcnt; ret++)
    {
      if (iov[ret].iov_base == NULL && iov[ret].iov_len != 0)
        {
          return -EFAULT;
        }
    }

  /* Is a driver registered? Does it support the write method?
   * If yes, then let the driver perform the write.
   */

  ret = -EBADF;
  inode = filep->f_inode;
  if (inode != NULL && inode->u.i_ops)
    {
      if (inode->u.i_ops->writev)
        {
          struct uio uio;

          ret = uio_init(&uio, iov, iovcnt);
          if (ret == 0)
            {
              ret = inode->u.i_ops->writev(filep, &uio);
            }
        }
      else if (inode->u.i_ops->write)
        {
          ret = file_writev_compat(filep, iov, iovcnt);
        }
    }

#ifdef CONFIG_FS_NOTIFY
  if (ret > 0)
    {
      notify_write(filep);
    }
#endif

  return ret;
}

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
  struct iovec iov;

  iov.iov_base = (FAR void *)buf;
  iov.iov_len = nbytes;

  return file_writev(filep, &iov, 1);
}

/****************************************************************************
 * Name: nx_writev
 *
 * Description:
 *  nx_writev() writes up to nytes bytes to the file referenced by the file
 *  descriptor fd from the buffer starting at buf.  nx_writev() is an
 *  internal OS function.  It is functionally equivalent to writev() except
 *  that:
 *
 *  - It does not modify the errno variable, and
 *  - It is not a cancellation point.
 *
 * Input Parameters:
 *   fd     - file descriptor to write to
 *   iov    - Data to write
 *   iovcnt - The number of vectors
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments withwrite() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_writev(int fd, FAR const struct iovec *iov, int iovcnt)
{
  FAR struct file *filep;
  ssize_t ret;

  /* First, get the file structure.
   * Note that file_get() will return the errno on failure.
   */

  ret = (ssize_t)file_get(fd, &filep);
  if (ret >= 0)
    {
      /* Perform the write operation using the file descriptor as an
       * index.  Note that file_writev() will return the errno on failure.
       */

      ret = file_writev(filep, iov, iovcnt);

      file_put(filep);
    }

  return ret;
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
  struct iovec iov;

  iov.iov_base = (void *)buf;
  iov.iov_len = nbytes;
  return nx_writev(fd, &iov, 1);
}

/****************************************************************************
 * Name: writev
 *
 * Description:
 *  writev() writes up to nytes bytes to the file referenced by the file
 *  descriptor fd from the buffer starting at buf.
 *
 * Input Parameters:
 *   fd     - file descriptor to write to
 *   iov    - Data to write
 *   iovcnt - The number of vectors
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

ssize_t writev(int fd, FAR const struct iovec *iov, int iovcnt)
{
  ssize_t ret;

  /* write() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_writev() do all of the work */

  ret = nx_writev(fd, iov, iovcnt);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
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
  struct iovec iov;

  iov.iov_base = (void *)buf;
  iov.iov_len = nbytes;
  return writev(fd, &iov, 1);
}
