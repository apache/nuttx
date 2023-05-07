/****************************************************************************
 * libs/libc/dirent/lib_closedir.c
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

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: closedir
 *
 * Description:
 *    The closedir() function closes the directory stream associated with
 *    'dirp'.  The directory stream descriptor 'dirp' is not available after
 *    this call.
 *
 * Input Parameters:
 *   dirp -- An instance of type DIR created by a previous call to opendir();
 *
 * Returned Value:
 *   The closedir() function returns 0 on success.  On error, -1 is
 *   returned, and errno is set appropriately.
 *
 ****************************************************************************/

int closedir(FAR DIR *dirp)
{
  int ret;
#ifdef CONFIG_FDSAN
  uint64_t tag;
#endif

  if (dirp == NULL)
    {
      set_errno(EBADF);
      return -1;
    }

#ifdef CONFIG_FDSAN
  tag = android_fdsan_create_owner_tag(ANDROID_FDSAN_OWNER_TYPE_DIR,
                                       (uintptr_t)dirp);
  ret = android_fdsan_close_with_tag(dirp->fd, tag);
#else
  ret = close(dirp->fd);
#endif

  lib_free(dirp);
  return ret;
}
