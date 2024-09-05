/****************************************************************************
 * libs/libc/misc/lib_ftok.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <errno.h>
#include <string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftok
 *
 * Description:
 *   Convert a pathname and a project identifier to a System V IPC key.
 *   The ftok() function uses the identity of the file named by the given
 *   pathname (which must refer to an existing, accessible file) and the
 *   least significant 8 bits of proj_id (which must be nonzero) to
 *   generate a key_t type System V IPC key, suitable for use with
 *   msgget(2), semget(2), or shmget(2).
 *
 * Input Parameters:
 *   pathname - identity of the file name
 *   proj_id  - The value that uniquely project identifies.
 *
 * Returned Value:
 *   On success, the generated key_t value is returned.
 *   On failure -1 is returned, with errno indicating the error as for the
 *   stat(2) system call.
 *
 ****************************************************************************/

key_t ftok(FAR const char *pathname, int proj_id)
{
  char fullpath[PATH_MAX] = CONFIG_LIBC_FTOK_VFS_PATH "/";
  struct stat st;

  strlcat(fullpath, pathname, sizeof(fullpath));
  if (stat(fullpath, &st) < 0 && get_errno() == ENOENT)
    {
      /* Directory not exist, let's create one for caller */

      if (mkdir(fullpath, S_IRWXU) < 0 ||
          stat(fullpath, &st) < 0)
        {
          return (key_t)-1;
        }
    }

  return ((key_t)proj_id << 24 |
          (key_t)(st.st_dev & 0xff) << 16 |
          (key_t)(st.st_ino & 0xffff));
}
