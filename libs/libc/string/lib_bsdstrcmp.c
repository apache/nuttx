/****************************************************************************
 * libs/libc/string/lib_bsdstrcmp.c
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

#if !defined(CONFIG_LIBC_ARCH_STRCMP) && defined(LIBC_BUILD_STRCMP)
#undef strcmp
no_builtin("strcmp")
nosanitize_address
int strcmp(FAR const char *cs, FAR const char *ct)
{
  /* If cs or ct are unaligned, then compare bytes. */

  if (!UNALIGNED(cs, ct))
    {
      FAR libc_data_t *a1 = (FAR libc_data_t *)cs;
      FAR libc_data_t *a2 = (FAR libc_data_t *)ct;

      while (*a1 == *a2)
        {
          if (DETECTNULL(*a1))
            {
              return 0;
            }

          a1++;
          a2++;
        }

      cs = (FAR char *)a1;
      ct = (FAR char *)a2;
    }
  else if (!UNALIGNED4(cs, ct))
    {
      FAR uint32_t *a1 = (FAR uint32_t *)cs;
      FAR uint32_t *a2 = (FAR uint32_t *)ct;

      while (*a1 == *a2)
        {
          if (DETECTNULL32(*a1))
            {
              return 0;
            }

          a1++;
          a2++;
        }

      cs = (FAR char *)a1;
      ct = (FAR char *)a2;
    }

  while (*cs != '\0' && *cs == *ct)
    {
      cs++;
      ct++;
    }

  return (*(FAR unsigned char *)cs) - (*(FAR unsigned char *)ct);
}
#endif
