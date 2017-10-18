/****************************************************************************
 * fs/vfs/fs_write.c
 *
 *   Copyright (C) 2007-2009, 2012-2014, 2016-2017 Gregory Nutt. All rights
 *     reserved.
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
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
# include <sys/socket.h>
#endif

#include <nuttx/cancelpt.h>
#include <nuttx/net/net.h>

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
 *   input paramters:
 *
 *  - It does not modify the errno variable,
 *  - It is not a cancellation point, and
 *  - It does not handle socket descriptors.
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

ssize_t file_write(FAR struct file *filep, FAR const void *buf, size_t nbytes)
{
  FAR struct inode *inode;

  /* Was this file opened for write access? */

  if ((filep->f_oflags & O_WROK) == 0)
    {
      return -EBADF;
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
 *   fd     - file descriptor (or socket descriptor) to write to
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
#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR struct file *filep;
#endif
  ssize_t ret;

  if (buf == NULL)
    {
      return -EINVAL;
    }

  /* Did we get a valid file descriptor? */

#if CONFIG_NFILE_DESCRIPTORS > 0
  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
#endif
    {
#if defined(CONFIG_NET_TCP) && CONFIG_NSOCKET_DESCRIPTORS > 0
      /* Write to a socket descriptor is equivalent to send with flags == 0. */

      ret = nx_send(fd, buf, nbytes, 0);
#else
      ret = -EBADF;
#endif
    }

#if CONFIG_NFILE_DESCRIPTORS > 0
  else
    {
      /* The descriptor is in the right range to be a file descriptor..
       * write to the file.  Note that fs_getfilep() will set the errno on
       * failure.
       */

      ret = (ssize_t)fs_getfilep(fd, &filep);
      if (ret >= 0)
        {
          /* Perform the write operation using the file descriptor as an
           * index.  Note that file_write() will set the errno on failure.
           */

          ret = file_write(filep, buf, nbytes);
        }
    }
#endif

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
 *   fd     - file descriptor (or socket descriptor) to write to
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
 *    defined maximum file size or the process' file size limit, or
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

  (void)enter_cancellation_point();

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
