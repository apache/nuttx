/****************************************************************************
 * libs/libc/string/lib_memcmp.c
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
/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* How many bytes are copied each iteration of the word copy loop. */

#define LBLOCKSIZE (sizeof(long))

/* Threshhold for punting to the byte copier. */

#define TOO_SMALL(len) ((len) < LBLOCKSIZE)

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMCMP) && defined(LIBC_BUILD_MEMCMP)
#undef memcmp /* See mm/README.txt */
no_builtin("memcmp")
int memcmp(FAR const void *s1, FAR const void *s2, size_t n)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR unsigned char *p1 = (FAR unsigned char *)s1;
  FAR unsigned char *p2 = (FAR unsigned char *)s2;
  FAR unsigned long *a1;
  FAR unsigned long *a2;

  /* If the size is too small, or either pointer is unaligned,
   * then we punt to the byte compare loop.  Hopefully this will
   * not turn up in inner loops.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(p1, p2))
    {
      /* Otherwise, load and compare the blocks of memory one
       * word at a time.
       */

      a1 = (FAR unsigned long *)p1;
      a2 = (FAR unsigned long *)p2;
      while (n >= LBLOCKSIZE)
        {
          if (*a1 != *a2)
            {
              break;
            }

          a1++;
          a2++;
          n -= LBLOCKSIZE;
        }

      /* check s mod LBLOCKSIZE remaining characters */

      p1 = (FAR unsigned char *)a1;
      p2 = (FAR unsigned char *)a2;
    }

  while (n--)
    {
      if (*p1 != *p2)
        {
          return *p1 - *p2;
        }

      p1++;
      p2++;
    }
#else
  FAR unsigned char *p1 = (FAR unsigned char *)s1;
  FAR unsigned char *p2 = (FAR unsigned char *)s2;

  while (n-- > 0)
    {
      if (*p1 < *p2)
        {
          return -1;
        }
      else if (*p1 > *p2)
        {
          return 1;
        }

      p1++;
      p2++;
    }
#endif

  return 0;
}
#endif
