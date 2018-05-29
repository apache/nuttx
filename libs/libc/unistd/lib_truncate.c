/****************************************************************************
 * libs/libc/unistd/lib_truncate.c
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

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: truncate
 *
 * Description:
 *   The truncate() function causes the regular file named by path to have
 *   a size of length bytes.
 *
 *   If the file previously was larger than length, the extra data is
 *   discarded.  If it was previously shorter than length, it is unspecified
 *   whether the file is changed or its size increased.  If the file is
 *   extended, the extended area appears as if it were zero-filled.

 *   With truncate(), the file must be open for writing; for truncate(),
 *   the process must have write permission for the file.
 *
 *   truncate() does not modify the file offset for any open file
 *   descriptions associated with the file.
 *
 * Input Parameters:
 *   path   - The path to the regular file to be truncated.
 *   length - The new length of the regular file.
 *
 * Returned Value:
 *    Upon successful completion, truncate() return 0s. Otherwise a -1 is
 *    returned, and errno is set to indicate the error.

 *    EINTR
 *      - A signal was caught during execution.
 *    EINVAL
 *      - The length argument was less than 0.
 *    EFBIG or EINVAL
 *      - The length argument was greater than the maximum file size.
 *    EIO
 *      - An I/O error occurred while reading from or writing to a file
 *        system.
 *    EACCES
 *      - A component of the path prefix denies search permission, or write
 *        permission is denied on the file.
 *    EISDIR
 *      - The named file is a directory.
 *    ELOOP
 *      - Too many symbolic links were encountered in resolving path.
 *    ENAMETOOLONG
 *      - The length of the specified pathname exceeds PATH_MAX bytes, or
 *        the length of a component of the pathname exceeds NAME_MAX bytes.
 *    ENOENT
 *      - A component of path does not name an existing file or path is an
 *        empty string.
 *    ENOTDIR
 *      - A component of the path prefix of path is not a directory.
 *    EROFS
 *      - he named file resides on a read-only file system.
 *    ENAMETOOLONG
 *      - Pathname resolution of a symbolic link produced an intermediate
 *        result whose length exceeds PATH_MAX.
 *
 ****************************************************************************/

int truncate(FAR const char *path, off_t length)
{
  int fd;
  int ret;

  DEBUGASSERT(path != NULL && length >= 0);

  /* Open the regular file at 'path' for write-only access */

  fd = open(path, O_WRONLY);
  if (fd < 0)
    {
      return ERROR;
    }

  /* Then let ftruncate() do the work */

  ret = ftruncate(fd, length);

  close(fd);
  return ret;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT */
