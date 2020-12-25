/****************************************************************************
 * libc/stdlib/lib_ancstr2bstr.c
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

#include <stdlib.h>
#include <string.h>

#include "libc.h"

#if CHAR_BIT != 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *ancstr2bstr(FAR const char *src, size_t maxlen)
{
  FAR char *dst;
  size_t len;

  len = strnlen(src, maxlen);
  dst = lib_malloc(B2C(len + 1));
  if (dst)
    {
      dst[B2C(len + 1) - 1] = 0;
      cmem2bmem(dst, 0, src, len);
    }

  return dst;
}

#endif
