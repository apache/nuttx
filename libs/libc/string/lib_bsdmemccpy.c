/****************************************************************************
 * libs/libc/string/lib_bsdmemccpy.c
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
 * Name: memccpy
 *
 * Description:
 *   The memccpy() function copies bytes from memory area s2 into s1,
 *   stopping after the first occurrence of byte c (converted to an unsigned
 *   char) is copied, or after n bytes are copied, whichever comes first. If
 *   copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The memccpy() function returns a pointer to the byte after the copy of c
 *   in s1, or a null pointer if c was not found in the first n bytes of s2.
 *
 ****************************************************************************/

#undef memccpy
FAR void *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n)
{
  FAR unsigned char *pout = (FAR unsigned char *)s1;
  FAR const unsigned char *pin = (FAR const unsigned char *)s2;
  unsigned char endchar = c & 0xff;

  /* If the size is small, or either pin or pout is unaligned,
   * then punt into the byte copy loop.  This should be rare.
   */

  if (!TOO_SMALL(n) && !UNALIGNED(pin, pout))
    {
      FAR libc_data_t *paligned_out = (FAR libc_data_t *)pout;
      FAR const libc_data_t *paligned_in = (FAR libc_data_t *)pin;
      libc_data_t mask = 0;
      unsigned int i;

      for (i = 0; i < LITTLEBLOCKSIZE; i++)
        {
          mask = (mask << 8) + endchar;
        }

      while (n >= LITTLEBLOCKSIZE)
        {
          libc_data_t buffer = (libc_data_t)(*paligned_in);
          buffer ^= mask;
          if (DETECTNULL(buffer))
            {
              break;
            }

          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE;
        }

      pout = (FAR unsigned char *)paligned_out;
      pin = (FAR unsigned char *)paligned_in;
    }
  else if (!TOO_SMALL4(n) && !UNALIGNED4(pin, pout))
    {
      FAR uint32_t *paligned_out = (FAR uint32_t *)pout;
      FAR const uint32_t *paligned_in = (FAR uint32_t *)pin;
      uint32_t mask = 0;
      unsigned int i;

      for (i = 0; i < LITTLEBLOCKSIZE4; i++)
        {
          mask = (mask << 8) + endchar;
        }

      while (n >= LITTLEBLOCKSIZE4)
        {
          uint32_t buffer = *paligned_in;
          buffer ^= mask;
          if (DETECTNULL32(buffer))
            {
              break;
            }

          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE4;
        }

      pout = (FAR unsigned char *)paligned_out;
      pin = (FAR unsigned char *)paligned_in;
    }

  while (n--)
    {
      if ((*pout++ = *pin++) == endchar)
        {
          return pout;
        }
    }

  return NULL;
}
