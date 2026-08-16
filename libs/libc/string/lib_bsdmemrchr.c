/****************************************************************************
 * libs/libc/string/lib_bsdmemrchr.c
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

#include <string.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memrchr
 *
 * Description:
 *   The memrchr() function locates the last occurrence of 'c' (converted to
 *   an unsigned char) in the initial 'n' bytes (each interpreted as
 *   unsigned char) of the object pointed to by s.
 *
 * Returned Value:
 *   The memrchr() function returns a pointer to the located byte, or a null
 *   pointer if the byte does not occur in the object.
 *
 ****************************************************************************/

#undef memrchr
FAR void *memrchr(FAR const void *s, int c, size_t n)
{
  FAR const unsigned char *src0 =
            (FAR const unsigned char *)s + n - 1;
  FAR libc_data_t *asrc;
  unsigned char d = c;
  libc_data_t mask;
  unsigned int i;

  while (UNALIGNED_X(src0 + 1))
    {
      if (!n--)
        {
          return NULL;
        }

      if (*src0 == d)
        {
          return (FAR void *)src0;
        }

      src0--;
    }

  if (!TOO_SMALL(n))
    {
      /* If we get this far, we know that n is large and src0 is
       * word-aligned.
       * The fast code reads the source one word at a time and only
       * performs the bytewise search on word-sized segments if they
       * contain the search character, which is detected by XORing
       * the word-sized segment with a word-sized block of the search
       * character and then detecting for the presence of NUL in the
       * result.
       */

      asrc = (FAR libc_data_t *)(uintptr_t)(src0 - LITTLEBLOCKSIZE + 1);
      mask = d << 8 | d;
      mask = mask << 16 | mask;
      for (i = 32; i < LITTLEBLOCKSIZE * 8; i <<= 1)
        {
          mask = (mask << i) | mask;
        }

      while (n >= LITTLEBLOCKSIZE)
        {
          if (DETECTCHAR(*asrc, mask))
            {
              break;
            }

          n -= LITTLEBLOCKSIZE;
          asrc--;
        }

      /* If there are fewer than LITTLEBLOCKSIZE characters left,
       * then we resort to the bytewise loop.
       */

      src0 = (FAR unsigned char *)asrc + LITTLEBLOCKSIZE - 1;
    }

  while (n--)
    {
      if (*src0 == d)
        {
          return (FAR void *)src0;
        }

      src0--;
    }

  return NULL;
}
