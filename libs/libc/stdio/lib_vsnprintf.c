/****************************************************************************
 * libs/libc/stdio/lib_vsnprintf.c
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
   *  been sufficiently large excluding the terminating null shall be
   *  returned, and [buf] may be a null pointer." -- opengroup.org
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
