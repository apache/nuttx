/****************************************************************************
 * libs/libc/string/lib_bsdstpcpy.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nonzero if either x or y is not aligned on a "long" boundary. */

#define UNALIGNED(x, y) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) | ((long)(uintptr_t)(y) & (sizeof(long) - 1)))

/* Macros for detecting endchar */

#if LONG_MAX == 2147483647
#  define DETECTNULL(x) (((x) - 0x01010101) & ~(x) & 0x80808080)
#elif LONG_MAX == 9223372036854775807
/* Nonzero if x (a long int) contains a NULL byte. */

#  define DETECTNULL(x) (((x) - 0x0101010101010101) & ~(x) & 0x8080808080808080)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stpcpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'dest'.
 *
 * Returned Value:
 *   The stpcpy() function returns a pointer to the terminating NUL
 *   character copied into the 'dest' buffer
 *
 ****************************************************************************/

#ifndef CONFIG_LIBC_ARCH_STPCPY
#undef stpcpy /* See mm/README.txt */
nosanitize_address
FAR char *stpcpy(FAR char *dest, FAR const char *src)
{
  FAR long *aligned_dst;
  FAR const long *aligned_src;

  /* If src or dest is unaligned, then copy bytes. */

  if (!UNALIGNED(src, dest))
    {
      aligned_dst = (FAR long *)dest;
      aligned_src = (FAR long *)src;

      /* src and dest are both "long int" aligned, try to do "long int"
       * sized copies.
       */

      while (!DETECTNULL(*aligned_src))
        {
          *aligned_dst++ = *aligned_src++;
        }

      dest = (FAR char *)aligned_dst;
      src = (FAR char *)aligned_src;
    }

  while ((*dest++ = *src++) != '\0');

  return --dest;
}
#endif
