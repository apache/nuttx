/****************************************************************************
 * libs/libc/stdio/lib_renameat.c
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

#include <stdio.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: renameat
 *
 * Description:
 *   The renameat() system call operates in exactly the same way as rename(),
 *   except for  the  differences described here.
 *
 *   If the pathname given in oldpath is relative, then  it  is  interpreted
 *   relative  to  the directory referred to by the file descriptor olddirfd
 *   (rather than relative to the current working directory of  the  calling
 *   process, as is done by rename() for a relative pathname).
 *
 *   If oldpath is relative and olddirfd is the special value AT_FDCWD, then
 *   oldpath is interpreted relative to the current working directory of the
 *   calling process (like rename()).
 *
 *   If oldpath is absolute, then olddirfd is ignored.
 *
 *   The interpretation of newpath is as for oldpath, except that a relative
 *   pathname is interpreted relative to the directory referred  to  by  the
 *   file descriptor newdirfd.
 *
 * Input Parameters:
 *   olddirfd - The file descriptor of old directory.
 *   oldpath  - A pointer to the old path
 *   newdirfd - The file descriptor of new directory.
 *   newpath  - A pointer to the new path
 *
 * Returned Value:
 *   Return zero on success, or -1 if an error occurred (in which case,
 *   errno is set appropriately).
 *
 ****************************************************************************/

int renameat(int olddirfd, FAR const char *oldpath,
             int newdirfd, FAR const char *newpath)
{
  char oldfullpath[PATH_MAX];
  char newfullpath[PATH_MAX];
  int ret;

  ret = lib_getfullpath(olddirfd, oldpath,
                        oldfullpath, sizeof(oldfullpath));
  if (ret >= 0)
    {
      ret = lib_getfullpath(newdirfd, newpath,
                            newfullpath, sizeof(newfullpath));
    }

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return rename(oldfullpath, newfullpath);
}
