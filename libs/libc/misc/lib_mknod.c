/****************************************************************************
 * libs/libc/misc/lib_mknod.c
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
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mknod
 *
 * Description:
 *   The mknod() function shall create a new file named by the pathname to
 *   which the argument path points.
 *
 *   The file type for path is OR'ed into the mode argument, and the
 *   application shall select one of the following symbolic constants:
 *
 *          S_IFIFO            FIFO-special
 *          S_IFCHR            Character-special (non-portable)
 *          S_IFDIR            Directory (non-portable)
 *          S_IFBLK            Block-special (non-portable)
 *          S_IFREG            Regular (non-portable)
 *
 *   The only portable use of mknod() is to create a FIFO-special file.
 *   If mode is not S_IFIFO or dev is not 0, the behavior of mknod() is
 *   unspecified.
 *
 *   The permissions for the new file are OR'ed into the mode argument.
 *
 * Input Parameters:
 *   pathname - The full path to node.
 *   mode     - File type and permission.
 *   dev      - Ignored.
 *
 * Returned Value:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/

int mknod(FAR const char *path, mode_t mode, dev_t dev)
{
  int ret = -1;

  UNUSED(dev);

  switch (mode & S_IFMT)
    {
#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
      case S_IFIFO:
        ret = mkfifo(path, mode & ~S_IFMT);
        break;
#endif

      case S_IFDIR:
        ret = mkdir(path, mode & ~S_IFMT);
        break;

      case S_IFREG:
        ret = creat(path, mode & ~S_IFMT);
        if (ret >= 0)
          {
            ret = close(ret);
          }
        break;

      default:
        set_errno(EINVAL);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mknodat
 *
 * Description:
 *   The mknodat() system call operates in exactly the same way as mknod(),
 *   except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like mknod()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd - The file descriptor of directory.
 *   path  - a pointer to the path.
 *   mode  - the access mode.
 *   dev   - Ignored.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int mknodat(int dirfd, FAR const char *path, mode_t mode, dev_t dev)
{
  char fullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(dirfd, path, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return mknod(fullpath, mode, dev);
}
