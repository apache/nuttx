/****************************************************************************
 * libs/libc/wchar/lib_wcsnrtombs.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <wchar.h>

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcsnrtombs
 *
 * Description:
 *   The 'wcsrtombs' function converts a string of wide characters
 *   indirectly pointed to by 'src' to a corresponding multibyte character
 *   string stored in the array pointed to by 'dst'.  No more than 'len'
 *   bytes are written to'dst'.
 *
 *   If 'dst' is NULL, no characters are stored.
 *
 *   If 'dst' is not NULL, the pointer pointed to by 'src' is updated to
 *   point to the character after the one that conversion stopped at.  If
 *   conversion stops because a null character is encountered, *'src' is set
 *   to NULL.
 *
 *   The mbstate_t argument, 'ps', is used to keep track of the shift state.
 *   If it is NULL, 'wcsrtombs' uses an internal, static mbstate_t object,
 *   which is initialized to the initial conversion state at program startup.
 *
 *   The 'wcsnrtombs' function behaves identically to 'wcsrtombs', except
 *   that conversion stops after reading at most 'nwc' characters from the
 *   buffer pointed to by 'src'.
 *
 * Returned Value:
 *   The 'wcsrtombs' and 'wcsnrtombs' functions return the number of bytes
 *   stored in the array pointed to by 'dst' (not including any terminating
 *   null), if successful, otherwise it returns (size_t)-1.
 *
 * Portability:
 *   'wcsrtombs' is defined by C99 standard.
 *   'wcsnrtombs' is defined by the POSIX.1-2008 standard.
 ****************************************************************************/

size_t wcsnrtombs(FAR char *dst, FAR const wchar_t **src, size_t nwc,
                  size_t len, FAR mbstate_t *ps)
{
  size_t i;

  if (dst == NULL)
    {
      for (i = 0; i < nwc; i++)
        {
          wchar_t wc = (*src)[i];

          if (wc < 0 || wc > 0xff)
            {
              set_errno(EILSEQ);
              return -1;
            }

          if (wc == L'\0')
            {
              return i;
            }
        }

      return i;
    }

  for (i = 0; i < nwc && i < len; i++)
    {
      wchar_t wc = (*src)[i];

      if (wc < 0 || wc > 0xff)
        {
          *src += i;
          set_errno(EILSEQ);
          return -1;
        }

      dst[i] = wc;
      if (wc == L'\0')
        {
          *src = NULL;
          return i;
        }
    }

  *src += i;
  return i;
}

#endif /* CONFIG_LIBC_WCHAR */
