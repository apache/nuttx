/****************************************************************************
 * libs/libc/dirent/lib_telldir.c
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
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telldir
 *
 * Description:
 *   The telldir() function returns the current location
 *   associated with the directory stream dirp.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *
 * Returned Value:
 *   On success, the telldir() function returns the current
 *   location in the directory stream.  On error, -1 is
 *   returned, and errno is set appropriately.
 *
 *   EBADF - Invalid directory stream descriptor dir
 *
 ****************************************************************************/

off_t telldir(FAR DIR *dirp)
{
  if (dirp != NULL)
    {
      return lseek(dirp->fd, 0, SEEK_CUR);
    }

  set_errno(EBADF);
  return -1;
}
