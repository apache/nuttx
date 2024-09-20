/****************************************************************************
 * libs/libc/dirent/lib_fdopendir.c
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

#include <sys/stat.h>

#include <dirent.h>
#include <errno.h>

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdopendir
 *
 * Description:
 *   An equivalent of opendir() except that this takes an open descriptor
 *   instead of a pathname.
 *
 * Input Parameters:
 *   fd -- the directory to open
 *
 * Returned Value:
 *   The fdopendir() function returns a pointer to the directory stream.
 *   On error, NULL is returned, and errno is set appropriately.
 *
 *   ENOTDIR - 'fd' is not a directory.
 *
 *   See opendir() for other errors.
 *
 ****************************************************************************/

FAR DIR *fdopendir(int fd)
{
  struct stat st;
  FAR DIR *dir;
  int ret;

  ret = fstat(fd, &st);
  if (ret == -1)
    {
      return NULL;
    }

  if (!S_ISDIR(st.st_mode))
    {
      set_errno(ENOTDIR);
      return NULL;
    }

  dir = lib_malloc(sizeof(*dir));
  if (dir == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  dir->fd = fd;

#ifdef CONFIG_FDSAN
  android_fdsan_exchange_owner_tag(fd, 0,
    android_fdsan_create_owner_tag(ANDROID_FDSAN_OWNER_TYPE_DIR,
                                   (uintptr_t)dir));
#endif

  return dir;
}
