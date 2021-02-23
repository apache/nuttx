/****************************************************************************
 * fs/vfs/fs_sendfile.c
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

#include <sys/sendfile.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/net/net.h>

#ifdef CONFIG_NET_SENDFILE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendfile
 *
 * Description:
 *   sendfile() copies data between one file descriptor and another.
 *   Used with file descriptors it basically just wraps a sequence of
 *   reads() and writes() to perform a copy.
 *
 *   If the destination descriptor is a socket, it gives a better
 *   performance than simple reds() and writes(). The data is read directly
 *   into the net buffer and the whole tcp window is filled if possible.
 *
 *   NOTE: This interface is *not* specified in POSIX.1-2001, or other
 *   standards.  The implementation here is very similar to the Linux
 *   sendfile interface.  Other UNIX systems implement sendfile() with
 *   different semantics and prototypes.  sendfile() should not be used
 *   in portable programs.
 *
 * Input Parameters:
 *   infd   - A file (or socket) descriptor opened for reading
 *   outfd  - A descriptor opened for writing.
 *   offset - If 'offset' is not NULL, then it points to a variable
 *            holding the file offset from which sendfile() will start
 *            reading data from 'infd'.  When sendfile() returns, this
 *            variable will be set to the offset of the byte following
 *            the last byte that was read.  If 'offset' is not NULL,
 *            then sendfile() does not modify the current file offset of
 *            'infd'; otherwise the current file offset is adjusted to
 *            reflect the number of bytes read from 'infd.'
 *
 *            If 'offset' is NULL, then data will be read from 'infd'
 *            starting at the current file offset, and the file offset
 *            will be updated by the call.
 *   count -  The number of bytes to copy between the file descriptors.
 *
 * Returned Value:
 *   If the transfer was successful, the number of bytes written to outfd is
 *   returned.  On error, -1 is returned, and errno is set appropriately.
 *   There error values are those returned by read() or write() plus:
 *
 *   EINVAL - Bad input parameters.
 *   ENOMEM - Could not allocated an I/O buffer
 *
 ****************************************************************************/

ssize_t sendfile(int outfd, int infd, off_t *offset, size_t count)
{
#ifdef CONFIG_NET_SENDFILE
  /* Check the destination file descriptor:  Is it a (probable) file
   * descriptor?  Check the source file:  Is it a normal file?
   */

  FAR struct socket *psock;

  psock = sockfd_socket(outfd);
  if (psock != NULL)
    {
      FAR struct file *filep;
      int ret;

      /* This appears to be a file-to-socket transfer.  Get the file
       * structure.
       */

      ret = fs_getfilep(infd, &filep);
      if (ret < 0)
        {
          set_errno(-ret);
          return ERROR;
        }

      DEBUGASSERT(filep != NULL);

      /* Then let psock_sendfile do the work. */

      ret = psock_sendfile(psock, filep, offset, count);
      if (ret >= 0 || get_errno() != ENOSYS)
        {
          return ret;
        }

      /* Fall back to the slow path if errno equals ENOSYS,
       * because psock_sendfile fail to optimize this transfer.
       */
    }
#endif

  /* No... then this is probably a file-to-file transfer.  The generic
   * lib_sendfile() can handle that case.
   */

  return lib_sendfile(outfd, infd, offset, count);
}

#endif /* CONFIG_NET_SENDFILE */
