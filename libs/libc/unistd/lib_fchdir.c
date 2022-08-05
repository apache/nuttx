/****************************************************************************
 * libs/libc/unistd/lib_fchdir.c
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
#include <limits.h>
#include <unistd.h>

#ifndef CONFIG_DISABLE_ENVIRON

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fchdir
 *
 * Description:
 *   The fchdir() function changes the current workint directory of the
 *   calling process to the directory specified in fd.
 *
 *   The fchdir() function is identical to chdir(); the only difference is
 *   that the directory is given as an open file diescriptor.
 *
 * Input Parameters:
 *   fd - The file descriptor is the one used internally by the directory
 *        stream.
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   EACCES
 *     Search permission was denied on the directory open on fd.
 *   EBADF
 *     fd is not a valid file descriptor.
 *
 ****************************************************************************/

int fchdir(int fd)
{
  char path[PATH_MAX];
  int ret;

  ret = fcntl(fd, F_GETPATH, path);
  if (ret < 0)
    {
      return ret;
    }

  return chdir(path);
}

#endif /* !CONFIG_DISABLE_ENVIRON */
