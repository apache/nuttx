/****************************************************************************
 * libs/libc/string/lib_strlen.c
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
#define UNALIGNED(x) ((long)(uintptr_t)(x) & (LBLOCKSIZE - 1))

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

#if !defined(CONFIG_LIBC_ARCH_STRLEN) && defined(LIBC_BUILD_STRLEN)
#undef strlen /* See mm/README.txt */
nosanitize_address
size_t strlen(FAR const char *s)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR const char *start = s;
  FAR unsigned long *aligned_addr;

  /* Align the pointer, so we can search a word at a time. */

  while (UNALIGNED(s))
    {
      if (!*s)
        {
          return s - start;
        }

      s++;
    }

  /* If the string is word-aligned, we can check for the presence of
   * a null in each word-sized block.
   */

  aligned_addr = (FAR unsigned long *)s;
  while (!DETECTNULL(*aligned_addr))
    {
      aligned_addr++;
    }

  /* Once a null is detected, we check each byte in that block for a
   * precise position of the null.
   */

  s = (FAR char *)aligned_addr;
  while (*s)
    {
      s++;
    }

  return s - start;
#else
  FAR const char *sc;
  for (sc = s; *sc != '\0'; ++sc);
  return sc - s;
#endif
}
#endif
