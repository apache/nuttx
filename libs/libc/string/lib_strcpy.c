/****************************************************************************
 * libs/libc/string/lib_strcpy.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRING_OPTIMIZE
/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strcpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'des'.
 *
 * Returned Value:
 *   The strcpy() function returns the 'dest' pointer
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRCPY) && defined(LIBC_BUILD_STRCPY)
#undef strcpy /* See mm/README.txt */
nosanitize_address
FAR char *strcpy(FAR char *dest, FAR const char *src)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR char *dst0 = dest;
  FAR const char *src0 = src;
  FAR unsigned long *aligned_dst;
  FAR const unsigned long *aligned_src;

  /* If SRC or DEST is unaligned, then copy bytes. */

  if (!UNALIGNED(src0, dst0))
    {
      aligned_dst = (FAR unsigned long *)dst0;
      aligned_src = (FAR unsigned long *)src0;

      /* SRC and DEST are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (!DETECTNULL(*aligned_src))
        {
          *aligned_dst++ = *aligned_src++;
        }

      dst0 = (FAR char *)aligned_dst;
      src0 = (FAR char *)aligned_src;
    }

  while ((*dst0++ = *src0++) != '\0');

  return dest;
#else
  FAR char *tmp = dest;
  while ((*dest++ = *src++) != '\0');
  return tmp;
#endif
}
#endif
