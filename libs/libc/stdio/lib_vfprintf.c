/****************************************************************************
 * libs/libc/stdio/lib_vfprintf.c
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

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int vfprintf(FAR FILE *stream, FAR const IPTR char *fmt, va_list ap)
{
  struct lib_stdoutstream_s stdoutstream;
  int  n = ERROR;

  /* Wrap the stream in a stream object and let lib_vsprintf
   * do the work.
   */

  lib_stdoutstream(&stdoutstream, stream);

  /* Hold the stream semaphore throughout the lib_vsprintf
   * call so that this thread can get its entire message out
   * before being pre-empted by the next thread.
   */

  flockfile(stream);
  n = lib_vsprintf(&stdoutstream.public, fmt, ap);
  funlockfile(stream);

  return n;
}
