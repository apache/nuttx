/****************************************************************************
 * libs/libc/string/lib_bsdstrlen.c
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

#define LBLOCKSIZE (sizeof(long))
#define UNALIGNED(x) ((long)(uintptr_t)(x) & (LBLOCKSIZE - 1))

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

#if !defined(CONFIG_LIBC_ARCH_STRLEN) && defined(LIBC_BUILD_STRLEN)
#undef strlen /* See mm/README.txt */
nosanitize_address
size_t strlen(FAR const char *s)
{
  FAR const char *start = s;
  FAR unsigned long *aligned_addr;

  /* Align the pointer, so we can search a word at a time. */

  while (UNALIGNED(s))
    {
      if (!*s)
        {
          return s - start;
        }

      s++;
    }

  /* If the string is word-aligned, we can check for the presence of
   * a null in each word-sized block.
   */

  aligned_addr = (FAR unsigned long *)s;
  while (!DETECTNULL(*aligned_addr))
    {
      aligned_addr++;
    }

  /* Once a null is detected, we check each byte in that block for a
   * precise position of the null.
   */

  s = (FAR char *)aligned_addr;
  while (*s)
    {
      s++;
    }

  return s - start;
}
#endif
