/****************************************************************************
 * libs/libc/string/lib_strchr.c
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

#define UNALIGNED(x) ((long)(uintptr_t)(x) & (sizeof(long) - 1))

/* How many bytes are loaded each iteration of the word copy loop. */

#define LBLOCKSIZE (sizeof(long))

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

#define DETECTCHAR(x, mask) (DETECTNULL((x) ^ (mask)))

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strchr
 *
 * Description:
 *   The strchr() function locates the first occurrence of 'c' (converted to
 *   a char) in the string pointed to by 's'. The terminating null byte is
 *   considered to be part of the string.
 *
 * Returned Value:
 *   Upon completion, strchr() returns a pointer to the byte, or a null
 *   pointer if the byte was not found.
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRCHR) && defined(LIBC_BUILD_STRCHR)
#undef strchr /* See mm/README.txt */
nosanitize_address
FAR char *strchr(FAR const char *s, int c)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR const unsigned char *s1 = (FAR const unsigned char *)s;
  FAR unsigned long *aligned_addr;
  unsigned char i = c;
  unsigned long mask;
  unsigned long j;

  /* Special case for finding 0. */

  if (!i)
    {
      while (UNALIGNED(s1))
        {
          if (!*s1)
            {
              return (FAR char *)s1;
            }

          s1++;
        }

      /* Operate a word at a time. */

      aligned_addr = (FAR unsigned long *)s1;
      while (!DETECTNULL(*aligned_addr))
        {
          aligned_addr++;
        }

      /* Found the end of string. */

      s1 = (FAR const unsigned char *)aligned_addr;
      while (*s1)
        {
          s1++;
        }

      return (FAR char *)s1;
    }

  /* All other bytes.  Align the pointer, then search a long at a time. */

  while (UNALIGNED(s1))
    {
      if (!*s1)
        {
          return NULL;
        }

      if (*s1 == i)
        {
          return (FAR char *)s1;
        }

      s1++;
    }

  mask = i;
  for (j = 8; j < LBLOCKSIZE * 8; j <<= 1)
    {
      mask = (mask << j) | mask;
    }

  aligned_addr = (FAR unsigned long *)s1;
  while (!DETECTNULL(*aligned_addr) && !DETECTCHAR(*aligned_addr, mask))
    {
      aligned_addr++;
    }

  /* The block of bytes currently pointed to by aligned_addr
   * contains either a null or the target char, or both.  We
   * catch it using the bytewise search.
   */

  s1 = (FAR unsigned char *)aligned_addr;

  while (*s1 && *s1 != i)
    {
      s1++;
    }

  if (*s1 == i)
    {
      return (FAR char *)s1;
    }
#else
  for (; ; s++)
    {
      if (*s == c)
        {
          return (FAR char *)s;
        }

      if (*s == '\0')
        {
          break;
        }
    }
#endif

  return NULL;
}
#endif
