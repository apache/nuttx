/****************************************************************************
 * libs/libc/misc/lib_getfullpath.c
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

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_getfullpath
 *
 * Description:
 *   Given the directory path from dirfd.
 *
 ****************************************************************************/

int lib_getfullpath(int dirfd, FAR const char *path, FAR char *fullpath)
{
  if (path == NULL || fullpath == NULL)
    {
      return -EINVAL;
    }
  else if (path[0] == '/')
    {
      /* The path is absolute, then dirfd is ignored. */

      strlcpy(fullpath, path, PATH_MAX);
      return 0;
    }

  if (dirfd == AT_FDCWD)
    {
      /* The dirfd is the special value AT_FDCWD, then pathname is
       * interpreted relative to the current working directory of the
       * calling process.
       */

      FAR char *pwd = "";

#ifndef CONFIG_DISABLE_ENVIRON
      pwd = getenv("PWD");
      if (pwd == NULL)
        {
          pwd = CONFIG_LIBC_HOMEDIR;
        }
#endif

      sprintf(fullpath, "%s/%s", pwd, path);
      return 0;
    }
  else
    {
      int ret;

      /* Get real directory path by dirfd */

      ret = fcntl(dirfd, F_GETPATH, fullpath);
      if (ret >= 0)
        {
          strlcat(fullpath, path, PATH_MAX);
        }

      return ret;
    }
}
