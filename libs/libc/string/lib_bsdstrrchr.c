/****************************************************************************
 * libs/libc/string/lib_bsdstrrchr.c
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
 * Public Functions
 ****************************************************************************/

/* The strrchr() function returns a pointer to the last
 * occurrence of the character c in the string s.
 */

#if !defined(CONFIG_LIBC_ARCH_STRRCHR) && defined(LIBC_BUILD_STRRCHR)
#undef strrchr /* See mm/README.txt */
FAR char *strrchr(FAR const char *s, int c)
{
  FAR const char *last = NULL;

  if (c)
    {
      while ((s = strchr(s, c)))
        {
          last = s;
          s++;
        }
    }
  else
    {
      last = strchr(s, c);
    }

  return (FAR char *)last;
}
#endif
