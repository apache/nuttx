/****************************************************************************
 * libs/libc/string/lib_bsdmemcpy.c
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
 * Name: memcpy
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMCPY) && defined(LIBC_BUILD_MEMCPY)
#undef memcpy
no_builtin("memcpy")
FAR void *memcpy(FAR void *dest, FAR const void *src, size_t n)
{
  FAR char *pout = dest;
  FAR const char *pin = src;

  /* Walk a pair that agrees about where a boundary falls up to it, so that
   * the word path below is reached even when the caller aligned neither
   * pointer.  Fewer than LITTLEBLOCKSIZE bytes are copied and the size was
   * tested against that first, so this cannot run past the end.
   */

  if (!TOO_SMALL(n) && !MISALIGNED(pin, pout) && UNALIGNED_X(pin))
    {
      do
        {
          *pout++ = *pin++;
          n--;
        }
      while (UNALIGNED_X(pin));
    }

  /* If the size is small, or either pin or pout is unaligned,
   * then punt into the byte copy loop.  This should be rare.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(pin, pout))
    {
      FAR libc_data_t *paligned_out = (FAR libc_data_t *)pout;
      FAR const libc_data_t *paligned_in = (FAR libc_data_t *)pin;

      /* Copy 4X libc_data_t words at a time if possible. */

      while (n >= BIGBLOCKSIZE)
        {
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          n -= BIGBLOCKSIZE;
        }

      /* Copy one libc_data_t word at a time if possible. */

      while (n >= LITTLEBLOCKSIZE)
        {
          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE;
        }

      pout = (FAR char *)paligned_out;
      pin = (FAR char *)paligned_in;
    }
  else if (!TOO_SMALL4(n) && !UNALIGNED4(pin, pout))
    {
      FAR uint32_t *paligned_out = (FAR uint32_t *)pout;
      FAR const uint32_t *paligned_in = (FAR uint32_t *)pin;

      /* Copy 4X uint32_t words at a time if possible. */

      while (n >= BIGBLOCKSIZE4)
        {
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          n -= BIGBLOCKSIZE4;
        }

      /* Copy one uint32_t word at a time if possible. */

      while (n >= LITTLEBLOCKSIZE4)
        {
          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE4;
        }

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
