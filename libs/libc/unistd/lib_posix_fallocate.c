/****************************************************************************
 * libs/libc/unistd/lib_posix_fallocate.c
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

#ifndef CONFIG_DISABLE_MOUNTPOINT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 * Returned Value:
 *   Upon successful completion, posix_fallocate() shall return zero;
 *   otherwise, an error number shall be returned to indicate the error.
 *
 ****************************************************************************/

int posix_fallocate(int fd, off_t offset, off_t len)
{
  struct stat st;

  if (offset < 0 || len < 0)
    {
      return EINVAL;
    }

  len += offset;
  if (len < 0)
    {
      return EFBIG;
    }

  if (fstat(fd, &st) != 0)
    {
      return get_errno();
    }

  if (st.st_size < len)
    {
      if (ftruncate(fd, len) != 0)
        {
          return get_errno();
        }
    }

  return 0;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT */
