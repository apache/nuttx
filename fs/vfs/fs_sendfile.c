/****************************************************************************
 * fs/vfs/fs_sendfile.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2013, 2017-2018 Gregory Nutt. All
 *     rights reserved.
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

  if ((unsigned int)outfd >= CONFIG_NFILE_DESCRIPTORS &&
      (unsigned int)infd < CONFIG_NFILE_DESCRIPTORS)
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

      /* Then let net_sendfile do the work. */

      ret = net_sendfile(outfd, filep, offset, count);
      if (ret >= 0 || get_errno() != ENOSYS)
        {
          return ret;
        }

      /* Fall back to the slow path if errno equals ENOSYS,
       * because net_sendfile fail to optimize this transfer.
       */
    }
#endif

  /* No... then this is probably a file-to-file transfer.  The generic
   * lib_sendfile() can handle that case.
   */

  return lib_sendfile(outfd, infd, offset, count);
}

#endif /* CONFIG_NET_SENDFILE */
