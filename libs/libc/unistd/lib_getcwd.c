/****************************************************************************
 * libs/libc/unistd/lib_getcwd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <limits.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getcwd
 *
 * Description:
 *   getcwd() function places the absolute pathname of the current working
 *   directory in the array pointed to by 'buf', and returns 'buf.' The
 *   pathname copied to the array shall contain no components that are
 *   symbolic links. The 'size' argument is the size in bytes of the
 *   character array pointed to by the 'buf' argument.  The current working
 *   directory is kernel state of the task group (chdir(), fchdir()); the
 *   environment variable PWD has no part in it.
 *
 *   As an extension to the POSIX.1-2001 standard, getcwd() allocates
 *   the buffer dynamically using lib_malloc if buf is NULL. In this case,
 *   the allocated buffer has the length size unless size is zero, when buf
 *   is allocated as big as necessary. The caller should free the
 *   returned buffer.
 *
 * Input Parameters:
 *   buf - a pointer to the location in which the current working directory
 *     pathname is returned.
 *   size - The size in bytes available at 'buf'
 *
 * Returned Value:
 *   Upon successful completion, getcwd() returns the 'buf' argument.
 *   Otherwise, getcwd() returns a null pointer and sets errno to indicate
 *   the error:
 *
 *   EINVAL
 *     The 'size' argument is 0 and the 'buf' argument is not NULL.
 *   ERANGE
 *     The size argument is greater than 0, but is smaller than the length
 *     of the current working directory pathname +1.
 *   ENOENT
 *     The current working directory was removed, or it is outside the
 *     chroot() root of the caller.
 *   ENOMEM
 *     Insufficient storage space is available.
 *
 ****************************************************************************/

FAR char *getcwd(FAR char *buf, size_t size)
{
  FAR char *alloc = NULL;
  int ret;

  /* Verify input parameters */

  if (buf != NULL && size == 0)
    {
      set_errno(EINVAL);
      return NULL;
    }

  if (buf == NULL)
    {
      if (size == 0)
        {
          size = PATH_MAX;
        }

      alloc = lib_malloc(size);
      if (alloc == NULL)
        {
          set_errno(ENOMEM);
          return NULL;
        }

      buf = alloc;
    }

  ret = nx_getcwd(buf, size);
  if (ret < 0)
    {
      lib_free(alloc);
      set_errno(-ret);
      return NULL;
    }

  return buf;
}

/****************************************************************************
 * Name: get_current_dir_name
 *
 * Description:
 *   get_current_dir_name() will allocate an buffer to hold the current
 *   working directory info, and this buffer will be returned to user.
 *   the user will be responsible to free the buffer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, get_current_dir_name() returns a pointer to a string
 *   containing the pathname of the current working directory.
 *   Otherwise, get_current_dir_name() returns a null pointer and
 *   sets errno to indicate the error.
 *   the errno can refer to: getcwd() function's comments.
 *
 ****************************************************************************/

FAR char *get_current_dir_name(void)
{
  return getcwd(NULL, 0);
}
