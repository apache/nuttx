/****************************************************************************
 * libs/libc/dirent/lib_dirfd.c
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dirfd
 *
 * Description:
 *   The dirfd() function returns the file descriptor associated
 *   with the directory stream dirp.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *           call to opendir();
 *
 * Returned Value:
 *   On success, a nonnegative file descriptor is returned.
 *   On error, -1 is returned, and errno is set to indicate
 *   the cause of the error.
 *
 *   EINVAL - dirp does not refer to a valid directory stream.
 *
 ****************************************************************************/

int dirfd(FAR DIR *dirp)
{
  if (dirp)
    {
      return dirp->fd;
    }

  set_errno(EINVAL);
  return -1;
}
