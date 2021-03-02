/****************************************************************************
 * libs/libc/stdio/lib_fputc.c
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
#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fputc
 ****************************************************************************/

int fputc(int c, FAR FILE *stream)
{
  unsigned char buf = (unsigned char)c;
  int ret;

  ret = lib_fwrite(&buf, 1, stream);
  if (ret > 0)
    {
      /* Flush the buffer if a newline is output */

      if (c == '\n' && (stream->fs_flags & __FS_FLAG_LBF) != 0)
        {
          ret = lib_fflush(stream, true);
          if (ret < 0)
            {
              return EOF;
            }
        }

      return c;
    }
  else
    {
      return EOF;
    }
}
