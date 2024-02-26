/****************************************************************************
 * libs/libc/string/lib_strcmp.c
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

#if !defined(CONFIG_LIBC_ARCH_STRCMP) && defined(LIBC_BUILD_STRCMP)
#undef strcmp /* See mm/README.txt */
nosanitize_address
int strcmp(FAR const char *cs, FAR const char *ct)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR unsigned long *a1;
  FAR unsigned long *a2;

  /* If cs or ct are unaligned, then compare bytes. */

  if (!UNALIGNED(cs, ct))
    {
      /* If cs and ct are word-aligned, compare them a word at a time. */

      a1 = (FAR unsigned long *)cs;
      a2 = (FAR unsigned long *)ct;
      while (*a1 == *a2)
        {
          /* To get here, *a1 == *a2, thus if we find a null in *a1,
           * then the strings must be equal, so return zero.
           */

          if (DETECTNULL(*a1))
            {
              return 0;
            }

          a1++;
          a2++;
        }

      /* A difference was detected in last few bytes of cs,
       * so search bytewise.
       */

      cs = (FAR char *)a1;
      ct = (FAR char *)a2;
    }

  while (*cs != '\0' && *cs == *ct)
    {
      cs++;
      ct++;
    }

  return (*(FAR unsigned char *)cs) - (*(FAR unsigned char *)ct);
#else
  register int result;
  for (; ; )
    {
      if ((result = (unsigned char)*cs - (unsigned char)*ct++) != 0 ||
          *cs++ == '\0')
        {
          break;
        }
    }

  return result;
#endif
}
#endif
