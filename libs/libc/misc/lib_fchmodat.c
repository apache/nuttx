/****************************************************************************
 * libs/libc/misc/lib_fchmodat.c
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
#include <fcntl.h>
#include <sys/stat.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fchmodat
 *
 * Description:
 *   The fchmodat() system call operates in exactly the same way as chmod(),
 *   except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like chmod()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd - The file descriptor of directory.
 *   path  - A pointer to the path.
 *   mode  - The access mode.
 *   flags - Ignored.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int fchmodat(int dirfd, FAR const char *path, mode_t mode, int flags)
{
  char fullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(dirfd, path, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  if ((flags & AT_SYMLINK_NOFOLLOW) != 0)
    {
      return lchmod(fullpath, mode);
    }

  return chmod(fullpath, mode);
}
