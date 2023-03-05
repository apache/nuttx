/****************************************************************************
 * libs/libc/unistd/lib_linkat.c
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

#include <errno.h>
#include <unistd.h>

#include "libc.h"

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: linkat
 *
 * Description:
 *   The linkat() system call operates in exactly the same way as link(),
 *   except for  the  differences described here.
 *
 *   If the pathname given in pathname is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor dirfd
 *   (rather than relative to the current working directory of the calling
 *    process)
 *   If  the  pathname  given in path1 is relative, then it is interpreted
 *   relative to the directory referred to by the file descriptor olddirfd
 *   (rather  than  relative to the current working directory of the calling
 *   process, as is done by link() for a relative pathname).
 *
 *   If path1 is relative and olddirfd is the special value AT_FDCWD, then
 *   oldpath is interpreted relative to the current working directory of the
 *   calling process (like link()).
 *
 *   If path1 is absolute, then olddirfd is ignored.
 *
 *   The interpretation of path2 is as for path1, except that a relative
 *   pathname is interpreted relative to the directory referred to by the
 *   file descriptor newdirfd.
 *
 * Input Parameters:
 *   olddirfd - The file descriptor of directory.
 *   path1    - Points to a pathname naming an existing file.
 *   newdirfd - The file descriptor of directory.
 *   path2    - Points to a pathname naming the new directory entry to be
 *              created.
 *   flags    - ignored.
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int linkat(int olddirfd, FAR const char *path1,
           int newdirfd, FAR const char *path2, int flags)
{
  char oldfullpath[PATH_MAX];
  char newfullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(olddirfd, path1,
                        oldfullpath, sizeof(oldfullpath));
  if (ret >= 0)
    {
      ret = lib_getfullpath(newdirfd, path2,
                            newfullpath, sizeof(newfullpath));
    }

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return link(oldfullpath, newfullpath);
}

#endif /* CONFIG_PSEUDOFS_SOFTLINKS */
