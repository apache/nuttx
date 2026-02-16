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

#define LBLOCKSIZE     (sizeof(long))
#define UNALIGNED(x)   ((long)(x) & (LBLOCKSIZE - 1))
#define TOO_SMALL(len) ((len) < LBLOCKSIZE)

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
  FAR unsigned long *aligned_addr;
  FAR char *s = (FAR char *)m;
  unsigned long buffer;
  int i;

  /* To avoid sign extension, copy C to an unsigned variable.  */

  while (UNALIGNED(s))
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

  if (!TOO_SMALL(n))
    {
      /* If we get this far, we know that n is large and s is word-aligned. */

      aligned_addr = (FAR unsigned long *)s;
      buffer = ((unsigned int)c << 8) | c;
      buffer |= (buffer << 16);
      for (i = 32; i < LBLOCKSIZE * 8; i <<= 1)
        {
          buffer = (buffer << i) | buffer;
        }

      /* Unroll the loop.  */

      while (n >= LBLOCKSIZE * 4)
        {
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          n -= 4 * LBLOCKSIZE;
        }

      while (n >= LBLOCKSIZE)
        {
          *aligned_addr++ = buffer;
          n -= LBLOCKSIZE;
        }

      /* Pick up the remainder with a bytewise loop.  */

      s = (FAR char *)aligned_addr;
    }

  while (n--)
    {
      *s++ = c;
    }

  return m;
}
#endif
