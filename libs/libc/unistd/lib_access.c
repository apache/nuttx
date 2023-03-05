/****************************************************************************
 * libs/libc/unistd/lib_access.c
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

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: access
 *
 * Description:
 *   The access() function shall check the file named by the pathname pointed
 *   to by the path argument for accessibility according to the bit pattern
 *   contained in amode, using the real user ID in place of the effective
 *   user ID and the real group ID in place of the effective group ID.
 *   As there are no users in NuttX, the function always succeeds.
 *
 * Input Parameters:
 *   path - a pointer to the path
 *   amode - the access mode
 *
 * Returned Value:
 *   Return OK (zero) if the caller can access the file with the required
 *   permission, otherwise return -1.
 *
 * Assumptions:
 *
 ****************************************************************************/

int access(FAR const char *path, int amode)
{
  struct stat s;

  if (stat(path, &s))
    {
      return -1;
    }

  if (s.st_mode & S_IFDIR)
    {
      return 0;
    }

  if (amode & W_OK)
    {
      if (s.st_mode & S_IWUSR)
        {
          return 0;
        }

      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: faccessat
 *
 * Description:
 *   The faccessat() system call operates in exactly the same way as
 *   access(), except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *
 *   If pathname is relative and dirfd is the special value AT_FDCWD, then
 *   pathname is interpreted relative to the current working directory of
 *   the calling process (like access()).
 *
 *   If pathname is absolute, then dirfd is ignored.
 *
 * Input Parameters:
 *   dirfd - The file descriptor of directory.
 *   path  - A pointer to the path.
 *   amode - The access mode.
 *   flags - Ignored.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int faccessat(int dirfd, FAR const char *path, int amode, int flags)
{
  char fullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(dirfd, path, fullpath, sizeof(fullpath));
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return access(fullpath, amode);
}
