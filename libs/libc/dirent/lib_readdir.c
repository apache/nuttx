/****************************************************************************
 * libs/libc/dirent/lib_readdir.c
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
 * Name: readdir
 *
 * Description:
 *   The readdir() function returns a pointer to a dirent structure
 *   representing the next directory entry in the directory stream pointed
 *   to by dir.  It returns NULL on reaching the end-of-file or if an error
 *   occurred.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous call to opendir();
 *
 * Returned Value:
 *   The readdir() function returns a pointer to a dirent structure, or NULL
 *   if an error occurs or end-of-file is reached.  On error, errno is set
 *   appropriately.
 *
 *   EBADF   - Invalid directory stream descriptor dir
 *
 ****************************************************************************/

FAR struct dirent *readdir(DIR *dirp)
{
  int ret;

  if (!dirp)
    {
      set_errno(EBADF);
      return NULL;
    }

  ret = read(dirp->fd, &dirp->entry, sizeof(struct dirent));
  if (ret <= 0)
    {
      return NULL;
    }

  return &dirp->entry;
}
