/****************************************************************************
 * libs/libc/stdio/lib_libsprintf.c
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
 * Name: lib_sprintf
 ****************************************************************************/

int lib_sprintf(FAR struct lib_outstream_s *obj, FAR const IPTR char *fmt,
                ...)
{
  va_list ap;
  int     n;

  /* Let lib_vsprintf do the real work */

  va_start(ap, fmt);
  n = lib_vsprintf(obj, fmt, ap);
  va_end(ap);
  return n;
}
