/****************************************************************************
 * libs/libc/string/lib_bsdstrchrnul.c
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

/****************************************************************************
 * Name: strchrnul
 *
 * Description:
 *   The strchrnul() function locates the first occurrence of 'c' (converted
 *   to a char) in the string pointed to by 's'. The terminating null byte is
 *   considered to be part of the string.
 *
 * Returned Value:
 *   Upon completion, strchrnul() returns a pointer to the byte, or a
 *   pointer to null if the byte was not found.
 *
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_STRCHRNUL) && defined(LIBC_BUILD_STRCHRNUL)
#undef strchrnul /* See mm/README.txt */
FAR char *strchrnul(FAR const char *s, int c)
{
  FAR char *s1 = strchr(s, c);

  return s1 ? s1 : (FAR char *)s + strlen(s);
}
#endif
