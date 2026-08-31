/****************************************************************************
 * libs/libc/misc/lib_statx.c
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
#include <string.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: statx
 *
 * Description:
 *   statx() is the Linux extended status interface.  It operates like
 *   fstatat(), but reports the result in the Linux struct statx layout.
 *
 *   If the pathname given in path is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *   process).
 *
 *   If path is relative and dirfd is the special value AT_FDCWD, then
 *   path is interpreted relative to the current working directory of the
 *   calling process (like stat()).
 *
 *   If path is absolute, then dirfd is ignored.
 *
 *   If AT_EMPTY_PATH is set in flags and path is an empty string, then
 *   the file referred to by dirfd itself is queried.
 *
 *   If AT_SYMLINK_NOFOLLOW is set in flags, then symbolic links are not
 *   followed (like lstat()).
 *
 *   The AT_STATX_SYNC_TYPE values and AT_NO_AUTOMOUNT are accepted but
 *   are no-ops: NuttX has no remote synchronization or automount
 *   semantics.  mask is a request hint only; this implementation always
 *   fills exactly STATX_BASIC_STATS, which Linux permits (the kernel may
 *   return more or fewer fields than requested).
 *
 * Input Parameters:
 *   dirfd - The file descriptor of the directory.
 *   path  - A pointer to the path.
 *   flags - AT_* flags as described above.
 *   mask  - Requested STATX_* fields (hint, may be ignored).
 *   buf   - The buffer to receive the extended file status.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int statx(int dirfd, FAR const char *path, int flags,
          unsigned int mask, FAR struct statx *buf)
{
  FAR char *fullpath;
  struct stat st;
  int ret;

  /* Sanity check the arguments (Linux reports EFAULT for NULL pointers) */

  if (buf == NULL || path == NULL)
    {
      set_errno(EFAULT);
      return ERROR;
    }

  if ((flags & ~(AT_SYMLINK_NOFOLLOW | AT_EMPTY_PATH | AT_NO_AUTOMOUNT |
                 AT_STATX_SYNC_TYPE)) != 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Obtain the stat data */

  if (path[0] != '\0')
    {
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
          ret = lstat(fullpath, &st);
        }
      else
        {
          ret = stat(fullpath, &st);
        }

      lib_put_pathbuffer(fullpath);
    }
  else if ((flags & AT_EMPTY_PATH) != 0)
    {
      ret = fstat(dirfd, &st);
    }
  else
    {
      set_errno(ENOENT);
      return ERROR;
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Translate struct stat into the Linux struct statx layout.  Fields
   * that NuttX cannot provide (birth time, mount ID, DIO alignment,
   * attributes) remain zero and are excluded from stx_mask.
   */

  memset(buf, 0, sizeof(*buf));

  buf->stx_mask          = STATX_BASIC_STATS;
  buf->stx_blksize       = st.st_blksize;
  buf->stx_nlink         = st.st_nlink;
  buf->stx_uid           = st.st_uid;
  buf->stx_gid           = st.st_gid;
  buf->stx_mode          = st.st_mode;
  buf->stx_ino           = st.st_ino;
  buf->stx_size          = st.st_size;
  buf->stx_blocks        = st.st_blocks;
  buf->stx_atime.tv_sec  = st.st_atim.tv_sec;
  buf->stx_atime.tv_nsec = st.st_atim.tv_nsec;
  buf->stx_mtime.tv_sec  = st.st_mtim.tv_sec;
  buf->stx_mtime.tv_nsec = st.st_mtim.tv_nsec;
  buf->stx_ctime.tv_sec  = st.st_ctim.tv_sec;
  buf->stx_ctime.tv_nsec = st.st_ctim.tv_nsec;
  buf->stx_rdev_major    = major(st.st_rdev);
  buf->stx_rdev_minor    = minor(st.st_rdev);
  buf->stx_dev_major     = major(st.st_dev);
  buf->stx_dev_minor     = minor(st.st_dev);

  return OK;
}
