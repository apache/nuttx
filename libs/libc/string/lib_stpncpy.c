/****************************************************************************
 * libs/libc/string/lib_stpncpy.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LIBC_STRING_OPTIMIZE
/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* How many bytes are loaded each iteration of the word copy loop. */

#define LBLOCKSIZE (sizeof(long))

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

#define TOO_SMALL(len) ((len) < sizeof(long))

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stpncpy
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
 *   If a NUL character is written to the destination, the stpncpy()
 *   function will return the address of the first such NUL character.
 *   Otherwise, it will return &dest[n]
 *
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_STPNCPY
#undef stpncpy /* See mm/README.txt */
FAR char *stpncpy(FAR char *dest, FAR const char *src, size_t n)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR char *ret = NULL;
  FAR long *aligned_dst;
  FAR const long *aligned_src;

  /* If src and dest is aligned and n large enough, then copy words. */

  if (!UNALIGNED(src, dest) && !TOO_SMALL(n))
    {
      aligned_dst = (FAR long *)dest;
      aligned_src = (FAR long *)src;

      /* src and dest are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (n >= LBLOCKSIZE && !DETECTNULL(*aligned_src))
        {
          n -= LBLOCKSIZE;
          *aligned_dst++ = *aligned_src++;
        }

      dest = (FAR char *)aligned_dst;
      src = (FAR char *)aligned_src;
    }

  while (n > 0)
    {
      --n;
      if ((*dest++ = *src++) == '\0')
        {
          ret = dest - 1;
          break;
        }
    }

  while (n-- > 0)
    {
      *dest++ = '\0';
    }

  return ret ? ret : dest;
#else
  FAR char *end = dest + n; /* End of dest buffer + 1 byte */
  FAR char *ret;            /* Value to be returned */

  /* Copy up n bytes, breaking out of the loop early if a NUL terminator is
   * encountered.
   */

  while ((dest != end) && (*dest = *src++) != '\0')
    {
      /* Increment the 'dest' pointer only if it does not refer to the
       * NUL terminator.
       */

      dest++;
    }

  /* Return the pointer to the NUL terminator (or to the end of the buffer
   * + 1).
   */

  ret = dest;

  /* Pad the remainder of the array pointer to 'dest' with NULs.  This
   * overwrites any previously copied NUL terminator.
   */

  while (dest != end)
    {
      *dest++ = '\0';
    }

  return ret;
#endif
}
#endif
