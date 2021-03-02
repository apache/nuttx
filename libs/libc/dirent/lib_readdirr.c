/****************************************************************************
 * libs/libc/dirent/lib_readdirr.c
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

#include <string.h>
#include <dirent.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readdir_r
 *
 * Description:
 *   The readdir() function returns a pointer to a dirent
 *   structure representing the next directory entry in the
 *   directory stream pointed to by dir.  It returns NULL on
 *   reaching the end-of-file or if an error occurred.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   entry -- The  storage  pointed to by entry must be large
 *     enough for a dirent with an array of char d_name
 *     members containing at least {NAME_MAX}+1 elements.
 *   result -- Upon successful return, the pointer returned
 *     at *result shall have the  same  value  as  the
 *     argument entry. Upon reaching the end of the directory
 *     stream, this pointer shall have the value NULL.
 *
 * Returned Value:
 *   If successful, the readdir_r() function return s zero;
 *   otherwise, an error number is returned to indicate the
 *   error.
 *
 *   EBADF   - Invalid directory stream descriptor dir
 *
 ****************************************************************************/

int readdir_r(FAR DIR *dirp, FAR struct dirent *entry,
              FAR struct dirent **result)
{
  struct dirent *tmp;

  /* NOTE: The following use or errno is *not* thread-safe */

  set_errno(0);
  tmp = readdir(dirp);
  if (!tmp)
    {
      int error = get_errno();
      if (!error)
        {
          if (result)
            {
              *result = NULL;
            }

          return 0;
        }
      else
        {
          return error;
        }
    }

  if (entry)
    {
      memcpy(entry, tmp, sizeof(struct dirent));
    }

  if (result)
    {
      *result = entry;
    }

  return 0;
}
