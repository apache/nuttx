/****************************************************************************
 * libs/libc/string/lib_bsdstrcpy.c
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
 * Name: strcpy
 *
 * Description:
 *   Copies the string pointed to by 'src' (including the terminating NUL
 *   character) into the array pointed to by 'des'.
 *
 * Returned Value:
 *   The strcpy() function returns the 'dest' pointer
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRCPY) && defined(LIBC_BUILD_STRCPY)
#undef strcpy
nosanitize_address
no_builtin("strcpy")
FAR char *strcpy(FAR char *dest, FAR const char *src)
{
  FAR char *dst0 = dest;
  FAR const char *src0 = src;
  FAR libc_data_t *aligned_dst;
  FAR const libc_data_t *aligned_src;

  /* If SRC or DEST is unaligned, then copy bytes. */

  if (!UNALIGNED(src0, dst0))
    {
      aligned_dst = (FAR libc_data_t *)dst0;
      aligned_src = (FAR libc_data_t *)src0;

      /* SRC and DEST are both "libc_data_t" aligned, try to do "libc_data_t"
       * sized copies.
       */

      while (!DETECTNULL(*aligned_src))
        {
          *aligned_dst++ = *aligned_src++;
        }

      dst0 = (FAR char *)aligned_dst;
      src0 = (FAR char *)aligned_src;
    }

  while ((*dst0++ = *src0++) != '\0');

  return dest;
}
#endif
