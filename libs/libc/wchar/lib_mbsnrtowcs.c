/****************************************************************************
 * libs/libc/wchar/lib_mbsnrtowcs.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <string.h>
#include <wchar.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mbsnrtowcs
 *
 * Description:
 *   The 'mbsrtowcs' function converts a sequence of multibyte characters
 *   pointed to indirectly by 'src' into a sequence of corresponding wide
 *   characters and stores at most 'len' of them in the wchar_t array pointed
 *   to by 'dst', until it encounters a terminating null character ('\0').
 *
 *   If 'dst' is NULL, no characters are stored.
 *
 *   If 'dst' is not NULL, the pointer pointed to by 'src' is updated to
 *   point to the character after the one that conversion stopped at.  If
 *   conversion stops because a null character is encountered, *'src' is set
 *   to NULL.
 *
 *   The mbstate_t argument, 'ps', is used to keep track of the shift state.
 *   If it is NULL, 'mbsrtowcs' uses an internal, static mbstate_t object,
 *   which is initialized to the initial conversion state at program startup.
 *
 *   The 'mbsnrtowcs' function behaves identically to 'mbsrtowcs', except
 *   that conversion stops after reading at most 'nms' bytes from the buffer
 *   pointed to by 'src'.
 *
 * Returned Value:
 *   The 'mbsrtowcs' and 'mbsnrtowcs' functions return the number of wide
 *   characters stored in the array pointed to by 'dst' if successful,
 *   otherwise it returns (size_t)-1.
 *
 * Portability:
 *   'mbsrtowcs' is defined by the C99 standard.
 *   'mbsnrtowcs' is defined by the POSIX.1-2008 standard.
 *
 ****************************************************************************/

size_t mbsnrtowcs(FAR wchar_t *dst, FAR const char **src, size_t nms,
                  size_t len, FAR mbstate_t *ps)
{
  size_t i;

  if (dst == NULL)
    {
      return strnlen(*src, nms);
    }

  for (i = 0; i < nms && i < len; i++)
    {
      dst[i] = (wchar_t)(*src)[i];
      if (dst[i] == L'\0')
        {
          *src = NULL;
          return i;
        }
    }

  *src += i;
  return i;
}

#endif /* CONFIG_LIBC_WCHAR */
