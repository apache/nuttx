/****************************************************************************
 * libs/libc/unistd/lib_readlinkat.c
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

#include <errno.h>
#include <unistd.h>

#include "libc.h"

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readlinkat
 *
 * Description:
 *   The readlinkat() system call operates in exactly the same way as
 *   readlink(), except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like readlink()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd   - The file descriptor of directory.
 *   path    - The full path to the symbolic link
 *   buf     - The user-provided buffer in which to return the path to the
 *             link target.
 *   bufsize - The size of 'buf'
 *
 * Returned Value:
 *   Upon successful completion, readlinkat() will return the count of bytes
 *   placed in the buffer. Otherwise, it will return a value of -1, leave the
 *   buffer unchanged, and set errno to indicate the error.
 *
 ****************************************************************************/

ssize_t readlinkat(int dirfd, FAR const char *path, FAR char *buf,
                   size_t bufsize)
{
  char fullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(dirfd, path, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return readlink(fullpath, buf, bufsize);
}

#endif /* CONFIG_PSEUDOFS_SOFTLINKS */
