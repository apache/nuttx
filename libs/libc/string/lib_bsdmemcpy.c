/****************************************************************************
 * libs/libc/string/lib_bsdmemcpy.c
 *
 * Copyright (c) 1994-2009  Red Hat, Inc. All rights reserved.
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the BSD License.   This program is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY expressed or implied,
 * including the implied warranties of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  A copy of this license is available at
 * http://www.opensource.org/licenses. Any Red Hat trademarks that are
 * incorporated in the source code or documentation are not subject to
 * the BSD License and may only be used or replicated with the express
 * permission of Red Hat, Inc.
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

/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* How many bytes are copied each iteration of the 4X unrolled loop. */

#define BIGBLOCKSIZE (sizeof(long) << 2)

/* How many bytes are copied each iteration of the word copy loop. */

#define LITTLEBLOCKSIZE (sizeof(long))

/* Threshhold for punting to the byte copier. */

#define TOO_SMALL(len) ((len) < BIGBLOCKSIZE)

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

  return dest;
}
#endif
