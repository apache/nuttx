/****************************************************************************
 * libs/libc/stdlib/lib_mkstemp.c
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
#include <nuttx/compiler.h>

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* mktemp() replaces at most six X's */

#define MKSTEMP_MAXXS    6

/* The number of names that are tried when the file exists */

#define MKSTEMP_RETRIES  238328  /* 62**3 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkostemps
 *
 * Description:
 *   Replace the X's (up to six) before the last suffixlen characters of
 *   path_template with characters that make the name unique, then create
 *   and open the file for reading and writing.  If another process created
 *   a file of that name in the meantime, try a new name.
 *
 * Input Parameters:
 *   path_template - The name of the file to create, with the X's.
 *   suffixlen     - The number of characters after the X's.
 *   flags         - More flags for open(), such as O_CLOEXEC or O_APPEND.
 *
 * Returned Value:
 *   An open file descriptor on success.  Otherwise -1 is returned and
 *   errno is set.
 *
 ****************************************************************************/

int mkostemps(FAR char *path_template, int suffixlen, int flags)
{
  char suffix0;
  size_t len = strlen(path_template);
  size_t end;
  size_t xlen;
  int retries;
  int fd;

  if (suffixlen < 0 || (size_t)suffixlen > len)
    {
      set_errno(EINVAL);
      return -1;
    }

  /* Count the X's before the suffix */

  end = len - suffixlen;
  for (xlen = 0; xlen < MKSTEMP_MAXXS && xlen < end &&
       path_template[end - xlen - 1] == 'X'; xlen++);

  for (retries = 0; retries < MKSTEMP_RETRIES; retries++)
    {
      /* mktemp() replaces the X's at the end of the string: cut the suffix
       * off while it runs.
       */

      memset(&path_template[end - xlen], 'X', xlen);
      suffix0 = path_template[end];
      path_template[end] = '\0';
      if (mktemp(path_template) == NULL)
        {
          path_template[end] = suffix0;
          return -1;
        }

      path_template[end] = suffix0;

      fd = open(path_template, O_RDWR | O_CREAT | O_EXCL |
                (flags & ~O_ACCMODE), 0600);
      if (fd >= 0 || get_errno() != EEXIST || xlen == 0)
        {
          return fd;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: mkostemp
 *
 * Description:
 *   As mkstemp(), with more flags for open().
 *
 ****************************************************************************/

int mkostemp(FAR char *path_template, int flags)
{
  return mkostemps(path_template, 0, flags);
}

/****************************************************************************
 * Name: mkstemps
 *
 * Description:
 *   As mkstemp(), with suffixlen characters after the X's.
 *
 ****************************************************************************/

int mkstemps(FAR char *path_template, int suffixlen)
{
  return mkostemps(path_template, suffixlen, 0);
}

/****************************************************************************
 * Name: mkstemp
 *
 * Description:
 *   The mkstemp() function replaces the contents of the string pointed to
 *   by path_template by a unique filename, and returns a file descriptor
 *   for the file open for reading and writing. The function thus prevents
 *   any possible race condition between testing whether the file exists and
 *   opening it for use. The string in path_template should look like a
 *   filename with six trailing 'X' s; mkstemp() replaces each 'X' with a
 *   character from the portable filename character set. The characters are
 *   chosen such that the resulting name does not duplicate the name of an
 *   existing file at the time of a call to mkstemp().
 *
 * Input Parameters:
 *   path_template - The base file name that will be modified to produce
 *     the unique file name.  This must be a full path beginning with /tmp.
 *     This function will modify only the first XXXXXX characters within
 *     that full path.
 *
 * Returned Value:
 *   Upon successful completion, mkstemp() returns an open file descriptor.
 *   Otherwise, -1 is returned if no suitable file could be created.
 *
 ****************************************************************************/

int mkstemp(FAR char *path_template)
{
  return mkostemps(path_template, 0, 0);
}
