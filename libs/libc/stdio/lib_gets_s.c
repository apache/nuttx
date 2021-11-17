/****************************************************************************
 * libs/libc/stdio/lib_gets_s.c
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
#include <stdint.h>
#include <unistd.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gets
 *
 * Description:
 *   gets() reads a line from stdin into the buffer pointed to by s until
 *   either a terminating newline or EOF, which it replaces with '\0'.  Reads
 *   at most n-1 characters from stdin into the array pointed to by str until
 *   new-line character, end-of-file condition, or read error.   The newline
 *   character, if encountered, is not saved in the array.  A NUL character
 *   is written immediately after the last character read into the array, or
 *   to str[0] if no characters were read.
 *
 *   If n is zero or is greater than RSIZE_MAX, a null character is written
 *   to str[0] but the function reads and discards characters from stdin
 *   until new-line character, end-of-file condition, or read error (not
 *   implemented).
 *
 *   If n-1 characters have been read, continues reading and discarding the
 *   characters from stdin until new-line character, end-of-file condition,
 *   or read error.
 *
 ****************************************************************************/

FAR char *gets_s(FAR char *s, rsize_t n)
{
  /* Handle the case where n is out of range as required */

  if (n < 1 || n > RSIZE_MAX)
    {
      /* Set n=1, i.e., room only for the NUL terminator */

      n = 1;
    }

  /* Then let lib_fgets() do the heavy lifting */

#ifdef CONFIG_FILE_STREAM
  return lib_fgets(s, n, stdin, false, true);
#else
  return lib_dgets(s, n, STDIN_FILENO, false, true);
#endif
}
