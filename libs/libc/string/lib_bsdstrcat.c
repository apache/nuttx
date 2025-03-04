/****************************************************************************
 * libs/libc/string/lib_bsdstrcat.c
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

#define ALIGNED(x) \
  (((long)(uintptr_t)(x) & (sizeof(long) - 1)) == 0)

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

#if !defined(CONFIG_LIBC_ARCH_STRCAT) && defined(LIBC_BUILD_STRCAT)
#undef strcat /* See mm/README.txt */
nosanitize_address
FAR char *strcat(FAR char *dest, FAR const char *src)
{
  FAR char *ret = dest;

  /* Skip over the data in dest as quickly as possible. */

  if (ALIGNED(dest))
    {
      FAR unsigned long *aligned_s1 = (FAR unsigned long *)dest;
      while (!DETECTNULL(*aligned_s1))
        {
          aligned_s1++;
        }

      dest = (FAR char *)aligned_s1;
    }

  while (*dest)
    {
      dest++;
    }

  /* dest now points to the its trailing null character, we can
   * just use strcpy to do the work for us now.
   * ?!? We might want to just include strcpy here.
   * Also, this will cause many more unaligned string copies because
   * dest is much less likely to be aligned.  I don't know if its worth
   * tweaking strcpy to handle this better.
   */

  strcpy(dest, src);

  return ret;
}
#endif
