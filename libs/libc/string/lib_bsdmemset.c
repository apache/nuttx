/****************************************************************************
 * libs/libc/string/lib_bsdmemset.c
 *
 * SPDX-License-Identifier: BSD
 * SPDX-FileCopyrightText: 1994-2009  Red Hat, Inc. All rights reserved
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memset
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMSET) && defined(LIBC_BUILD_MEMSET)
#undef memset
no_builtin("memset")
FAR void *memset(FAR void *m, int c, size_t n)
{
  FAR libc_data_t *aligned_addr;
  FAR char *s = (FAR char *)m;
  libc_data_t buffer;
  int i;

  /* To avoid sign extension, copy C to an unsigned variable.  */

  while (UNALIGNED(s, 0))
    {
      if (n--)
        {
          *s++ = c;
        }
      else
        {
          return m;
        }
    }

  buffer  = (uint8_t)c;
  buffer |= (buffer << 8);
  buffer |= (buffer << 16);
  for (i = 32; i < LITTLEBLOCKSIZE * 8; i <<= 1)
    {
      buffer = (buffer << i) | buffer;
    }

  if (!TOO_SMALL(n))
    {
      /* If we get this far, we know that n is large and s is word-aligned. */

      aligned_addr = (FAR libc_data_t *)s;

      /* Unroll the loop.  */

      while (n >= LITTLEBLOCKSIZE * 4)
        {
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          n -= 4 * LITTLEBLOCKSIZE;
        }

      while (n >= LITTLEBLOCKSIZE)
        {
          *aligned_addr++ = buffer;
          n -= LITTLEBLOCKSIZE;
        }

      /* Pick up the remainder with a bytewise loop.  */

      s = (FAR char *)aligned_addr;
    }

  /* Tail: here s is libc_data_t-aligned and n < LITTLEBLOCKSIZE.
   * Fill with aligned stores of decreasing width - no unaligned access,
   * no overlap, no over-write.
   */

  if (LITTLEBLOCKSIZE > 8 && n >= 8)
    {
      *(FAR uint64_t *)s = (uint64_t)buffer;
      s += 8;
      n -= 8;
    }

  if (n >= 4)
    {
      *(FAR uint32_t *)s = (uint32_t)buffer;
      s += 4;
      n -= 4;
    }

  if (n >= 2)
    {
      *(FAR uint16_t *)s = (uint16_t)buffer;
      s += 2;
      n -= 2;
    }

  if (n)
    {
      *s = (uint8_t)buffer;
    }

  return m;
}
#endif
