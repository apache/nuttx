/****************************************************************************
 * libs/libc/string/lib_bsdstpncpy.c
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
 * Name: stpncpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.  strncpy() will not
 *   copy more than 'n' bytes from 'src' to 'dest' array (including the
 *   NUL terminator).
 *
 *   If the array pointed to by 'src' is a string that is shorter than 'n'
 *   bytes, NUL characters will be appended to the copy in the array
 *   pointed to by 'dest', until 'n' bytes in all are written.
 *
 *   If copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   If a NUL character is written to the destination, the stpncpy()
 *   function will return the address of the first such NUL character.
 *   Otherwise, it will return &dest[n]
 *
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_STPNCPY
#undef stpncpy
no_builtin("stpncpy")
FAR char *stpncpy(FAR char *dest, FAR const char *src, size_t n)
{
  FAR char *ret = NULL;

  /* Walk a pair that agrees about where a boundary falls up to it, so that
   * the word path below is reached even when the caller aligned neither
   * pointer.  The terminator is left for the byte loop, which also pads.
   * Fewer than LITTLEBLOCKSIZE bytes are copied and n was tested against
   * that first, so n cannot run out here.
   */

  if (!MISALIGNED(src, dest) && !TOO_SMALL(n) && UNALIGNED_X(src))
    {
      while (*src != '\0')
        {
          *dest++ = *src++;
          n--;
          if (!UNALIGNED_X(src))
            {
              break;
            }
        }
    }

  /* If src and dest is aligned and n large enough, then copy words. */

  if (!UNALIGNED(src, dest) && !TOO_SMALL(n))
    {
      FAR libc_data_t *aligned_dst = (FAR libc_data_t *)dest;
      FAR const libc_data_t *aligned_src = (FAR libc_data_t *)src;

      while (n >= LITTLEBLOCKSIZE && !DETECTNULL(*aligned_src))
        {
          n -= LITTLEBLOCKSIZE;
          *aligned_dst++ = *aligned_src++;
        }

      dest = (FAR char *)aligned_dst;
      src = (FAR char *)aligned_src;
    }
  else if (!UNALIGNED4(src, dest) && !TOO_SMALL4(n))
    {
      FAR uint32_t *aligned_dst = (FAR uint32_t *)dest;
      FAR const uint32_t *aligned_src = (FAR uint32_t *)src;

      while (n >= LITTLEBLOCKSIZE4 && !DETECTNULL32(*aligned_src))
        {
          n -= LITTLEBLOCKSIZE4;
          *aligned_dst++ = *aligned_src++;
        }

      dest = (FAR char *)aligned_dst;
      src = (FAR char *)aligned_src;
    }

  while (n > 0)
    {
      --n;
      if ((*dest++ = *src++) == '\0')
        {
          ret = dest - 1;
          break;
        }
    }

  while (n-- > 0)
    {
      *dest++ = '\0';
    }

  return ret ? ret : dest;
}
#endif
