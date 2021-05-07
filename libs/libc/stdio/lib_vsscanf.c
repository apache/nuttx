/****************************************************************************
 * libs/libc/stdio/lib_vsscanf.c
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
 * Name: vsscanf
 ****************************************************************************/

int vsscanf(FAR const char *buf, FAR const IPTR char *fmt, va_list ap)
{
  struct lib_meminstream_s meminstream;
  int n;

  /* Initialize a memory stream to freadm from the buffer */

  lib_meminstream((FAR struct lib_meminstream_s *)&meminstream, buf,
                  LIB_BUFLEN_UNKNOWN);

  /* Then let lib_vscanf do the real work */

  n = lib_vscanf((FAR struct lib_instream_s *)&meminstream.public, NULL,
                  fmt, ap);
  return n;
}
