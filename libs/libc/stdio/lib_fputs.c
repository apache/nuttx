/****************************************************************************
 * libs/libc/stdio/lib_fputs.c
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

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fputs
 *
 * Description:
 *   fputs() writes the string s to stream, without its trailing '\0'.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ROMGETC)
int fputs(FAR const IPTR char *s, FAR FILE *stream)
{
  int nput;
  int ret;
  char ch;

  /* Write the string.  Loop until the null terminator is encountered */

  for (nput = 0, ch = up_romgetc(s); ch; nput++, s++, ch = up_romgetc(s))
    {
      /* Write the next character to the stream buffer */

      ret = lib_fwrite(&ch, 1, stream);
      if (ret <= 0)
        {
          return EOF;
        }

      /* Flush the buffer if a newline was written to the buffer */

      if (ch == '\n' && (stream->fs_flags & __FS_FLAG_LBF) != 0)
        {
          ret = lib_fflush(stream, true);
          if (ret < 0)
            {
              return EOF;
            }
        }
    }

  return nput;
}

#else
int fputs(FAR const IPTR char *s, FAR FILE *stream)
{
  int nput;

  /* If line buffering is enabled, then we will have to output one character
   * at a time, checking for a newline character each time.
   */

  if ((stream->fs_flags & __FS_FLAG_LBF) != 0)
    {
      int ret;

      /* Write the string.  Loop until the null terminator is encountered */

      for (nput = 0; *s; nput++, s++)
        {
          /* Write the next character to the stream buffer */

          ret = lib_fwrite(s, 1, stream);
          if (ret <= 0)
            {
              return EOF;
            }

          /* Flush the buffer if a newline was written to the buffer */

          if (*s == '\n')
            {
              ret = lib_fflush(stream, true);
              if (ret < 0)
                {
                  return EOF;
                }
            }
        }
    }

  /* We can write the whole string in one operation without line buffering */

  else
    {
      int ntowrite;

      /* Get the length of the string. */

      ntowrite = strlen(s);
      if (ntowrite == 0)
        {
          return 0;
        }

      /* Write the string */

      nput = lib_fwrite(s, ntowrite, stream);
      if (nput < 0)
        {
          return EOF;
        }
    }

  return nput;
}
#endif
