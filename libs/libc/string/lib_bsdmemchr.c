/****************************************************************************
 * libs/libc/string/lib_bsdmemchr.c
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

#define UNALIGNED(x) ((long)(uintptr_t)(x) & (sizeof(long) - 1))

/* How many bytes are loaded each iteration of the word copy loop. */

#define LBLOCKSIZE (sizeof(long))

/* Threshhold for punting to the bytewise iterator. */

#define TOO_SMALL(len) ((len) < LBLOCKSIZE)

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

/* DETECTCHAR returns nonzero if (long)x contains the byte used
 * to fill (long)mask.
 */

#define DETECTCHAR(x, mask) (DETECTNULL((x) ^ (mask)))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memchr
 *
 * Description:
 *   The memchr() function locates the first occurrence of 'c' (converted to
 *   an unsigned char) in the initial 'n' bytes (each interpreted as
 *   unsigned char) of the object pointed to by s.
 *
 * Returned Value:
 *   The memchr() function returns a pointer to the located byte, or a null
 *   pointer if the byte does not occur in the object.
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMCHR) && defined(LIBC_BUILD_MEMCHR)
#undef memchr /* See mm/README.txt */
FAR void *memchr(FAR const void *s, int c, size_t n)
{
  FAR const unsigned char *p = (FAR const unsigned char *)s;
  FAR unsigned long *asrc;
  unsigned char d = c;
  unsigned long mask;
  unsigned int i;

  while (UNALIGNED(p))
    {
      if (!n--)
        {
          return NULL;
        }

      if (*p == d)
        {
          return (FAR void *)p;
        }

      p++;
    }

  if (!TOO_SMALL(n))
    {
      /* If we get this far, we know that n is large and p is
       * word-aligned.
       * The fast code reads the source one word at a time and only
       * performs the bytewise search on word-sized segments if they
       * contain the search character, which is detected by XORing
       * the word-sized segment with a word-sized block of the search
       * character and then detecting for the presence of NUL in the
       * result.
       */

      asrc = (FAR unsigned long *)p;
      mask = d << 8 | d;
      mask = mask << 16 | mask;
      for (i = 32; i < LBLOCKSIZE * 8; i <<= 1)
        {
          mask = (mask << i) | mask;
        }

      while (n >= LBLOCKSIZE)
        {
          if (DETECTCHAR(*asrc, mask))
            {
              break;
            }

          n -= LBLOCKSIZE;
          asrc++;
        }

      /* If there are fewer than LBLOCKSIZE characters left,
       * then we resort to the bytewise loop.
       */

      p = (FAR unsigned char *)asrc;
    }

  while (n--)
    {
      if (*p == d)
        {
          return (FAR void *)p;
        }

      p++;
    }

  return NULL;
}
#endif
