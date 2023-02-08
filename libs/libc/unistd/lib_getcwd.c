/****************************************************************************
 * libs/libc/unistd/lib_getcwd.c
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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "libc.h"

#ifndef CONFIG_DISABLE_ENVIRON

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getwcd
 *
 * Description:
 *   getcwd() function places the absolute pathname of the current working
 *   directory in the array pointed to by 'buf', and returns 'buf.' The
 *   pathname copied to the array shall contain no components that are
 *   symbolic links. The 'size' argument is the size in bytes of the
 *   character array pointed to by the 'buf' argument.
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
 *   EACCES
 *     Read or search permission was denied for a component of the pathname.
 *   ENOMEM
 *     Insufficient storage space is available.
 *
 ****************************************************************************/

FAR char *getcwd(FAR char *buf, size_t size)
{
  char *pwd;

  /* Verify input parameters */

  if (buf && size == 0)
    {
      set_errno(EINVAL);
      return NULL;
    }

  if (size == 0)
    {
      size = PATH_MAX + 1;
    }

  /* If no working directory is defined, then default to the home directory */

  pwd = getenv("PWD");
  if (pwd == NULL)
    {
      pwd = CONFIG_LIBC_HOMEDIR;
    }

  /* Verify that the cwd will fit into the user-provided buffer */

  if (strlen(pwd) + 1 > size)
    {
      set_errno(ERANGE);
      return NULL;
    }

  if (buf == NULL)
    {
      buf = lib_malloc(size);
      if (!buf)
        {
          set_errno(ENOMEM);
          return NULL;
        }
    }

  /* Copy the cwd to the user buffer */

  strlcpy(buf, pwd, size);
  return buf;
}
#endif /* !CONFIG_DISABLE_ENVIRON */
