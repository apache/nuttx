/****************************************************************************
 * libs/libc/string/lib_bsdstrlen.c
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

#if !defined(CONFIG_LIBC_ARCH_STRLEN) && defined(LIBC_BUILD_STRLEN)
#undef strlen
no_builtin("strlen")
nosanitize_address
size_t strlen(FAR const char *s)
{
  FAR const char *start = s;
  FAR libc_data_t *aligned_addr;

  /* Align the pointer, so we can search a word at a time. */

  while (UNALIGNED_X(s))
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

  aligned_addr = (FAR libc_data_t *)s;
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
