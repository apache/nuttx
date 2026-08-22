/****************************************************************************
 * libs/libc/string/lib_bsdmemcmp.c
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

#if !defined(CONFIG_LIBC_ARCH_MEMCMP) && defined(LIBC_BUILD_MEMCMP)
#undef memcmp
no_builtin("memcmp")
int memcmp(FAR const void *s1, FAR const void *s2, size_t n)
{
  FAR unsigned char *p1 = (FAR unsigned char *)s1;
  FAR unsigned char *p2 = (FAR unsigned char *)s2;

  /* Walk a pair that agrees about where a boundary falls up to it, so that
   * the word path below is reached even when the caller aligned neither
   * pointer.  A difference found on the way stops the walk and is reported
   * by the byte loop.  Fewer than LITTLEBLOCKSIZE bytes are compared and
   * the size was tested against that first, so this cannot run past the
   * end.
   */

  if (!TOO_SMALL(n) && !MISALIGNED(p1, p2) && UNALIGNED_X(p1))
    {
      while (*p1 == *p2)
        {
          p1++;
          p2++;
          n--;
          if (!UNALIGNED_X(p1))
            {
              break;
            }
        }
    }

  /* If the size is too small, or either pointer is unaligned,
   * then we punt to the byte compare loop.  Hopefully this will
   * not turn up in inner loops.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(p1, p2))
    {
      FAR libc_data_t *a1 = (FAR libc_data_t *)p1;
      FAR libc_data_t *a2 = (FAR libc_data_t *)p2;

      while (n >= LITTLEBLOCKSIZE)
        {
          if (*a1 != *a2)
            {
              break;
            }

          a1++;
          a2++;
          n -= LITTLEBLOCKSIZE;
        }

      p1 = (FAR unsigned char *)a1;
      p2 = (FAR unsigned char *)a2;
    }
  else if (!TOO_SMALL4(n) && !UNALIGNED4(p1, p2))
    {
      FAR uint32_t *a1 = (FAR uint32_t *)p1;
      FAR uint32_t *a2 = (FAR uint32_t *)p2;

      while (n >= LITTLEBLOCKSIZE4)
        {
          if (*a1 != *a2)
            {
              break;
            }

          a1++;
          a2++;
          n -= LITTLEBLOCKSIZE4;
        }

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

  return 0;
}
#endif
