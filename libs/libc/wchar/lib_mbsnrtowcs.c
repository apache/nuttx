/****************************************************************************
 * libs/libc/wchar/lib_mbsnrtowcs.c
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
#include <string.h>
#include <wchar.h>

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
