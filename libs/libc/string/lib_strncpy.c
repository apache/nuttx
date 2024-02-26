/****************************************************************************
 * libs/libc/string/lib_strncpy.c
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
#include <sys/types.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRING_OPTIMIZE

#define LBLOCKSIZE (sizeof(long))

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

#define TOO_SMALL(len) ((len) < sizeof(long))

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strncpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.  strncpy() will not
 *   copy more than 'n' bytes from 'src' to 'dest' array (including the
 *   NUL terminator).
 *
 *   If the array pointed to by 'src' is a string that is shorter than 'n'
 *   bytes, NUL characters will be appended to the copy in the array
 *   pointed to by 'dest', until 'n' bytes in all are written.
 *
 *   If copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The strncpy() function returns the pointer to 'dest'
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRNCPY) && defined(LIBC_BUILD_STRNCPY)
#undef strncpy /* See mm/README.txt */
nosanitize_address
FAR char *strncpy(FAR char *dest, FAR const char *src, size_t n)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR char *dst0 = dest;
  FAR const char *src0 = src;
  FAR long *aligned_dst;
  FAR const long *aligned_src;

  /* If src and dest is aligned and n large enough, then copy words. */

  if (!UNALIGNED(src0, dst0) && !TOO_SMALL(n))
    {
      aligned_dst = (FAR long *)dst0;
      aligned_src = (FAR long *)src0;

      /* src and dest are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (n >= LBLOCKSIZE && !DETECTNULL(*aligned_src))
        {
          n -= LBLOCKSIZE;
          *aligned_dst++ = *aligned_src++;
        }

      dst0 = (FAR char *)aligned_dst;
      src0 = (FAR char *)aligned_src;
    }

  while (n > 0)
    {
      --n;
      if ((*dst0++ = *src0++) == '\0')
        {
          break;
        }
    }

  while (n-- > 0)
    {
      *dst0++ = '\0';
    }

  return dest;
#else
  FAR char *ret = dest;     /* Value to be returned */
  FAR char *end = dest + n; /* End of dest buffer + 1 byte */

  /* Copy up n bytes, breaking out of the loop early if a NUL terminator is
   * encountered.
   */

  while ((dest != end) && (*dest++ = *src++) != '\0')
    {
    }

  /* Note that there may be no NUL terminator in 'dest' */

  /* Pad the remainder of the array pointer to 'dest' with NULs */

  while (dest != end)
    {
      *dest++ = '\0';
    }

  return ret;
#endif
}
#endif
