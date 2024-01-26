/****************************************************************************
 * libs/libc/string/lib_memcpy.c
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

/* How many bytes are copied each iteration of the 4X unrolled loop. */

#define BIGBLOCKSIZE (sizeof(long) << 2)

/* How many bytes are copied each iteration of the word copy loop. */

#define LITTLEBLOCKSIZE (sizeof(long))

/* Threshhold for punting to the byte copier. */

#define TOO_SMALL(len) ((len) < BIGBLOCKSIZE)

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memcpy
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMCPY) && defined(LIBC_BUILD_MEMCPY)
#undef memcpy /* See mm/README.txt */
no_builtin("memcpy")
FAR void *memcpy(FAR void *dest, FAR const void *src, size_t n)
{
#ifdef CONFIG_LIBC_STRING_OPTIMIZE
  FAR char *pout = dest;
  FAR const char *pin = src;
  FAR long *paligned_out;
  FAR const long *paligned_in;

  /* If the size is small, or either pin or pout is unaligned,
   * then punt into the byte copy loop.  This should be rare.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(pin, pout))
    {
      paligned_out = (FAR long *)pout;
      paligned_in = (FAR long *)pin;

      /* Copy 4X long words at a time if possible. */

      while (n >= BIGBLOCKSIZE)
        {
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          n -= BIGBLOCKSIZE;
        }

      /* Copy one long word at a time if possible. */

      while (n >= LITTLEBLOCKSIZE)
        {
          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE;
        }

      /* Pick up any residual with a byte copier. */

      pout = (FAR char *)paligned_out;
      pin = (FAR char *)paligned_in;
    }

  while (n--)
    {
      *pout++ = *pin++;
    }
#else
  FAR unsigned char *pout = (FAR unsigned char *)dest;
  FAR unsigned char *pin  = (FAR unsigned char *)src;
  while (n-- > 0)
    {
      *pout++ = *pin++;
    }
#endif

  return dest;
}
#endif
