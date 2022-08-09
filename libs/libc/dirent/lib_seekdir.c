/****************************************************************************
 * libs/libc/dirent/lib_seekdir.c
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
#include <unistd.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seekdir
 *
 * Description:
 *   The seekdir() function sets the location in the directory stream from
 *   which the next readdir() call will start.  seekdir() should be used with
 *   an offset returned by telldir().
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   offset -- offset to seek to
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void seekdir(FAR DIR *dirp, off_t offset)
{
  if (dirp != NULL)
    {
      lseek(dirp->fd, offset, SEEK_SET);
    }
  else
    {
      set_errno(EBADF);
    }
}
