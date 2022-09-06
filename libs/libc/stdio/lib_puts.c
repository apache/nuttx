/****************************************************************************
 * libs/libc/stdio/lib_puts.c
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
#include <unistd.h>
#include <sys/uio.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: puts
 *
 * Description:
 *   puts() writes the string s and a trailing newline to stdout.
 *
 ****************************************************************************/

int puts(FAR const IPTR char *s)
{
#ifdef CONFIG_FILE_STREAM
  FILE *stream = stdout;
  int nwritten;
  int nput = EOF;
  int ret;

  /* Write the string (the next two steps must be atomic) */

  lib_take_lock(stream);

  /* Write the string without its trailing '\0' */

  nwritten = fputs(s, stream);
  if (nwritten > 0)
    {
      /* Followed by a newline */

      char newline = '\n';
      ret = lib_fwrite(&newline, 1, stream);
      if (ret > 0)
        {
          nput = nwritten + 1;

          /* Flush the buffer after the newline is output if line buffering
           * is enabled.
           */

          if ((stream->fs_flags & __FS_FLAG_LBF) != 0)
            {
              ret = lib_fflush(stream, true);
              if (ret < 0)
                {
                  nput = EOF;
                }
            }
        }
    }

  lib_give_lock(stdout);
  return nput;
#else
  size_t len = strlen(s);
  struct iovec iov[2];

  iov[0].iov_base = (FAR void *)s;
  iov[0].iov_len  = len;
  iov[1].iov_base = "\n";
  iov[1].iov_len  = 1;

  return writev(STDOUT_FILENO, iov, 2) == ++len ? len : EOF;
#endif
}
