/****************************************************************************
 * libs/libc/dirent/lib_nftw.c
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

#include <ftw.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
call_nftw(FAR char *path, nftw_cb_t fn, int flags, int base,
          int level, FAR const struct stat *buf, int info)
{
  struct FTW ftw =
  {
    base, level
  };

  int r;

#ifndef CONFIG_DISABLE_ENVIRON
  if (flags & FTW_CHDIR)
    {
      if (base > 1)
        {
          path[base - 1] = '\0';
          r = chdir(path);
          path[base - 1] = '/';
        }
      else
        {
          r = chdir("/");
        }

      if (r < 0)
        {
          return r;
        }
    }
#endif

  r = fn(path, buf, info, &ftw);

#ifndef CONFIG_DISABLE_ENVIRON
  if (flags & FTW_CHDIR)
    {
      lib_restoredir();
    }
#endif

  return r;
}

static int
do_nftw(FAR char *path, nftw_cb_t fn, int fdlimit, int flags, int level)
{
  FAR DIR *dir = NULL;
  struct stat buf;
  size_t base;
  size_t j;
  int info;
  int r;

  j = strlen(path);
  while (j > 1 && path[j - 1] == '/')
    {
      path[--j] = '\0';
    }

  base = j - 1;
  while (base > 0 && path[base - 1] != '/')
    {
      --base;
    }

  r = flags & FTW_PHYS ? lstat(path, &buf) : stat(path, &buf);
  if (r < 0)
    {
      if (!(flags & FTW_PHYS) &&
          get_errno() == ENOENT && !lstat(path, &buf))
        {
          info = FTW_SLN;
        }
      else if (get_errno() == EACCES)
        {
          info = FTW_NS;
        }
      else
        {
          return -1;
        }
    }
  else if (S_ISDIR(buf.st_mode))
    {
      if (flags & FTW_DEPTH)
        {
          info = FTW_DP;
        }
      else
        {
          info = FTW_D;
        }
    }
  else if (S_ISLNK(buf.st_mode))
    {
      if (flags & FTW_PHYS)
        {
          info = FTW_SL;
        }
      else
        {
          info = FTW_SLN;
        }
    }
  else
    {
      info = FTW_F;
    }

  if (info == FTW_D || info == FTW_DP)
    {
      dir = opendir(path);
      if (dir)
        {
          if (fdlimit <= 0)
            {
              closedir(dir);
              dir = NULL;
            }
        }
      else if (get_errno() == EACCES)
        {
          info = FTW_DNR;
        }
      else
        {
          return -1;
        }
    }

  if (!(flags & FTW_DEPTH))
    {
      r = call_nftw(path, fn, flags, base, level, &buf, info);
      if (r)
        {
          return r;
        }
    }

  if (dir)
    {
      FAR struct dirent *de;
      size_t l = j;

      if (path[j - 1] != '/')
        {
          path[j++] = '/';
        }

      while ((de = readdir(dir)))
        {
          if (de->d_name[0] == '.' && (!de->d_name[1] ||
              (de->d_name[1] == '.' && !de->d_name[2])))
            {
              continue;
            }

          if (strlen(de->d_name) > PATH_MAX - j)
            {
              set_errno(ENAMETOOLONG);
              closedir(dir);
              return -1;
            }

          strcpy(path + j, de->d_name);
          r = do_nftw(path, fn, fdlimit - 1, flags, level + 1);
          if (r)
            {
              closedir(dir);
              return r;
            }
        }

      path[l] = '\0';
      closedir(dir);
    }

  if (flags & FTW_DEPTH)
    {
      r = call_nftw(path, fn, flags, base, level, &buf, info);
      if (r)
        {
          return r;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nftw(FAR const char *path, nftw_cb_t fn, int fdlimit, int flags)
{
  char pathbuf[PATH_MAX + 1];

  strlcpy(pathbuf, path, sizeof(pathbuf));

  return do_nftw(pathbuf, fn, fdlimit, flags, 0);
}
