/****************************************************************************
 * libs/libc/stdio/lib_vfscanf.c
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
       * this thread can get its entire message out before being pre-empted
       * by the next thread.
       */

      flockfile(stream);

      n = lib_vscanf(&stdinstream.public, &lastc, fmt, ap);

      /* The lib_vscanf function reads always one character more, this
       * character needs to be written back.
       */

      if (lastc != EOF)
        {
          ungetc(lastc, stream);
        }

      funlockfile(stream);
    }

  return n;
}
