/****************************************************************************
 * libs/libc/string/lib_stpcpy.c
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
 * Name: stpcpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.
 *
 * Returned Value:
 *   The stpcpy() function returns a pointer to the terminating NUL
 *   character copied into the 'dest' buffer
 *
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_STPCPY
#undef stpcpy /* See mm/README.txt */
nosanitize_address
FAR char *stpcpy(FAR char *dest, FAR const char *src)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR long *aligned_dst;
  FAR const long *aligned_src;

  /* If src or dest is unaligned, then copy bytes. */

  if (!UNALIGNED(src, dest))
    {
      aligned_dst = (FAR long *)dest;
      aligned_src = (FAR long *)src;

      /* src and dest are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (!DETECTNULL(*aligned_src))
        {
          *aligned_dst++ = *aligned_src++;
        }

      dest = (FAR char *)aligned_dst;
      src = (FAR char *)aligned_src;
    }
#endif

  while ((*dest++ = *src++) != '\0');

  return --dest;
}
#endif
