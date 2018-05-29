/****************************************************************************
 * libs/libc/stdio/lib_vsnprintf.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsnprintf
 ****************************************************************************/

int vsnprintf(FAR char *buf, size_t size, FAR const IPTR char *format,
              va_list ap)
{
  union
  {
    struct lib_outstream_s    nulloutstream;
    struct lib_memoutstream_s memoutstream;
  } u;

  FAR struct lib_outstream_s *stream;
  int n;

  /* "If the value of [size] is zero on a call to vsnprintf(), nothing shall
   *  be written, the number of bytes that would have been written had [size]
   *  been sufficiently large excluding the terminating null shall be returned,
   *  and [buf] may be a null pointer." -- opengroup.org
   */

  if (size > 0)
    {
      /* Initialize a memory stream to write to the buffer */

      lib_memoutstream(&u.memoutstream, buf, size);
      stream = &u.memoutstream.public;
    }
  else
    {
      /* Use a null stream to get the size of the buffer */

      lib_nulloutstream(&u.nulloutstream);
      stream = &u.nulloutstream;
    }

  /* Then let lib_vsprintf do the real work */

  n = lib_vsprintf(stream, format, ap);
  return n;
}
