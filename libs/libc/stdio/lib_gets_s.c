/****************************************************************************
 * libs/libc/stdio/lib_gets_s.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>

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
 *   character, if encountered, is not saved in the arraay.  A NUL character
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

  return lib_fgets(s, (size_t)n, stdin, false, true);
}
