/****************************************************************************
 * libs/libc/wchar/lib_swprintf.c
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
#include <stdarg.h>
#include <stdio.h>
#include <wchar.h>
#include "libc.h"

#include <nuttx/streams.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * swprintf
 ****************************************************************************/

int swprintf(FAR wchar_t *buf, size_t maxlen, FAR const wchar_t *fmt, ...)
{
  struct lib_memoutstream_s memoutstream;
  va_list ap;
  int n;

  /* Initialize a memory stream to write to the buffer */

  lib_memoutstream((FAR struct lib_memoutstream_s *)&memoutstream,
                   (FAR char *)buf, maxlen);

  /* Then let lib_vsprintf do the real work */

  va_start(ap, fmt);
  n = lib_vsprintf((FAR struct lib_outstream_s *)&memoutstream.public,
                   (FAR const char *)fmt, ap);
  va_end(ap);

  return n;
}

#endif /* CONFIG_LIBC_WCHAR */
