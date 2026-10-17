/****************************************************************************
 * libs/libc/misc/lib_fstatat.c
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

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fstatat
 *
 * Description:
 *   The fstatat() system call operates in exactly the same way as stat(),
 *   except for the differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like stat()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 *   If pathname is an empty string and flags contains AT_EMPTY_PATH, then
 *   the call operates on the file referred to by dirfd (like fstat()).  If
 *   dirfd is AT_FDCWD, the call operates on the current working directory.
 *
 * Input Parameters:
 *   dirfd - The file descriptor of directory.
 *   path  - A pointer to the path.
 *   buf   - The buf to get file status.
 *   flags - AT_SYMLINK_NOFOLLOW and AT_EMPTY_PATH are supported.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int fstatat(int dirfd, FAR const char *path, FAR struct stat *buf,
            int flags)
{
  FAR char *fullpath;
  int ret;

  if (path != NULL && path[0] == '\0' && (flags & AT_EMPTY_PATH) != 0)
    {
      if (dirfd != AT_FDCWD)
        {
          return fstat(dirfd, buf);
        }

      path = ".";
    }

  fullpath = lib_get_pathbuffer();
  if (fullpath == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  ret = lib_getfullpath(dirfd, path, fullpath, PATH_MAX);
  if (ret < 0)
    {
      lib_put_pathbuffer(fullpath);
      set_errno(-ret);
      return ERROR;
    }

  if ((flags & AT_SYMLINK_NOFOLLOW) != 0)
    {
      ret = lstat(fullpath, buf);
    }
  else
    {
      ret = stat(fullpath, buf);
    }

  lib_put_pathbuffer(fullpath);
  return ret;
}
