/****************************************************************************
 * libs/libc/unistd/lib_truncate.c
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
