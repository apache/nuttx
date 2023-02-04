/****************************************************************************
 * libs/libc/misc/lib_openat.c
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

#include <stdarg.h>
#include <errno.h>
#include <fcntl.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: openat
 *
 * Description:
 *   The openat() system call operates in exactly the same way as open(),
 *   except for the differences:
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *   process).
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process.
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd  - The file descriptor of directory.
 *   path   - a pointer to the path
 *   oflags - the flag of open.
 *   amode  - the access mode
 *
 * Returned Value:
 *   Return the new file descriptor (a nonnegative integer), or -1 if an
 *   error occurred (in which case, errno is set appropriately).
 *
 ****************************************************************************/

int openat(int dirfd, FAR const char *path, int oflags, ...)
{
  char fullpath[PATH_MAX];
  mode_t mode = 0;
  int ret;

  ret = lib_getfullpath(dirfd, path, fullpath);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  if ((oflags & O_CREAT) != 0)
    {
      va_list ap;

      va_start(ap, oflags);
      mode = va_arg(ap, mode_t);
      va_end(ap);
    }

  return open(fullpath, oflags, mode);
}
