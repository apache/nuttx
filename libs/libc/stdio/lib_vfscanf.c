/****************************************************************************
 * libs/libc/stdio/lib_vfscanf.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Johannes Shock
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

#include <nuttx/streams.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfscanf
 ****************************************************************************/

int vfscanf(FAR FILE *stream, FAR const IPTR char *fmt, va_list ap)
{
  struct lib_stdinstream_s stdinstream;
  int n = ERROR;
  int lastc;

  if (stream)
    {
      /* Wrap the stream in a stream object and let lib_vscanf do the work. */

      lib_stdinstream(&stdinstream, stream);

      /* Hold the stream semaphore throughout the lib_vscanf call so that
       * this thread can get its entire message out before being pre-empted by
       * the next thread.
       */

      lib_take_semaphore(stream);

      n = lib_vscanf(&stdinstream.public, &lastc, fmt, ap);

      /* The lib_vscanf function reads always one character more, this
       * character needs to be written back.
       */

      if (lastc != EOF)
        {
          ungetc(lastc, stream);
        }

      lib_give_semaphore(stream);
    }

  return n;
}
