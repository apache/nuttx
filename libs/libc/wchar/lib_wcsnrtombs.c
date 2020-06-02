/****************************************************************************
 * libs/libc/wchar/lib_wcsnrtombs.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
