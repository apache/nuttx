/****************************************************************************
 * libs/libc/unistd/lib_fallocate.c
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

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <sys/stat.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fallocate
 *
 * Description:
 *   fallocate() is a Linux-specific interface that manipulates the file
 *   space for the file referred to by fd.  It is the single implementation
 *   shared by posix_fallocate(), which is a thin wrapper around it.
 *
 *   This is a truncate-based emulation:  with mode equal to zero it
 *   ensures that the file is at least offset+len bytes long by extending
 *   it via ftruncate() when necessary, which satisfies the default
 *   allocation semantics of both interfaces.  It is implementation-defined
 *   whether the underlying file system reserves media blocks eagerly.
 *
 *   FALLOC_FL_KEEP_SIZE is accepted as a no-op:  after validating the
 *   arguments and the file descriptor, the call succeeds without changing
 *   the file.  A truncate-based emulation cannot allocate space beyond
 *   the current end of file without changing st_size, so the keep-size
 *   semantics cannot be honored in general.
 *
 *   All other mode flags (FALLOC_FL_PUNCH_HOLE, FALLOC_FL_COLLAPSE_RANGE,
 *   FALLOC_FL_ZERO_RANGE, FALLOC_FL_INSERT_RANGE, FALLOC_FL_UNSHARE_RANGE)
 *   are not supported and fail with EOPNOTSUPP.
 *
 * Input Parameters:
 *   fd     - File descriptor of the regular file to be manipulated.
 *   mode   - Operation mode; zero or FALLOC_FL_KEEP_SIZE only.
 *   offset - Start of the byte range to allocate.
 *   len    - Length of the byte range to allocate.
 *
 * Returned Value:
 *   Upon successful completion, fallocate() returns zero; otherwise -1
 *   is returned and errno is set to indicate the error.
 *
 *   EINVAL      - offset or len was less than 0.
 *   EFBIG       - offset+len exceeds the maximum file size.
 *   EOPNOTSUPP  - mode contains flags other than FALLOC_FL_KEEP_SIZE.
 *   EBADF       - fd is not a valid file descriptor (via fstat()).
 *   Plus any error that fstat() or ftruncate() may report.
 *
 ****************************************************************************/

int fallocate(int fd, int mode, off_t offset, off_t len)
{
  struct stat st;

  if (offset < 0 || len < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (mode & ~FALLOC_FL_KEEP_SIZE)
    {
      set_errno(EOPNOTSUPP);
      return ERROR;
    }

  len += offset;
  if (len < 0)
    {
      set_errno(EFBIG);
      return ERROR;
    }

  if (fstat(fd, &st) != 0)
    {
      return ERROR;
    }

  if (mode & FALLOC_FL_KEEP_SIZE)
    {
      return OK;
    }

  if (st.st_size < len)
    {
      if (ftruncate(fd, len) != 0)
        {
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: posix_fallocate
 *
 * Description:
 *  The posix_fallocate() function shall ensure that any required storage for
 *  regular file data starting at offset and continuing for len bytes is
 *  allocated on the file system storage media. If posix_fallocate() returns
 *  successfully, subsequent writes to the specified file data shall not fail
 *  due to the lack of free space on the file system storage media.
 *
 *  If the offset+len is beyond the current file size, then posix_fallocate()
 *  shall adjust the file size to offset+len. Otherwise, the file size shall
 *  not be changed.
 *
 *  It is implementation-defined whether a previous posix_fadvise() call
 *  influences allocation strategy.
 *
 *  Space allocated via posix_fallocate() shall be freed by a successful call
 *  to creat() or open() that truncates the size of the file. Space allocated
 *  via posix_fallocate() may be freed by a successful call to ftruncate()
 *  that reduces the file size to a size smaller than offset+ len.
 *
 * Input Parameters:
 *   fd     - File descriptor of the regular file to be manipulated.
 *   offset - Start of the byte range to allocate.
 *   len    - Length of the byte range to allocate.
 *
 * Returned Value:
 *   Upon successful completion, posix_fallocate() shall return zero;
 *   otherwise, an error number shall be returned to indicate the error.
 *
 ****************************************************************************/

int posix_fallocate(int fd, off_t offset, off_t len)
{
  int ret = fallocate(fd, 0, offset, len);

  return ret < 0 ? get_errno() : 0;
}
