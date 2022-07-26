/****************************************************************************
 * libs/libc/dirent/lib_opendir.c
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

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opendir
 *
 * Description:
 *   The  opendir() function opens a directory stream corresponding to the
 *   directory name, and returns a pointer to the directory stream. The
 *   stream is positioned at the first entry in the directory.
 *
 * Input Parameters:
 *   path -- the directory to open
 *
 * Returned Value:
 *   The opendir() function returns a pointer to the directory stream.  On
 *   error, NULL is returned, and errno is set appropriately.
 *
 *   EACCES  - Permission denied.
 *   EMFILE  - Too many file descriptors in use by process.
 *   ENFILE  - Too many files are currently open in the
 *             system.
 *   ENOENT  - Directory does not exist, or name is an empty
 *             string.
 *   ENOMEM  - Insufficient memory to complete the operation.
 *   ENOTDIR - 'path' is not a directory.
 *
 ****************************************************************************/

FAR DIR *opendir(FAR const char *path)
{
  FAR DIR *dir;
  int fd;

  dir = lib_malloc(sizeof(*dir));
  if (dir == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  fd = open(path, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0)
    {
      lib_free(dir);
      return NULL;
    }

  dir->fd = fd;
  return dir;
}
