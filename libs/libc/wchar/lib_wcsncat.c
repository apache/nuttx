/****************************************************************************
 * libs/libc/wchar/lib_wcsncat.c
 *
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2005-2014 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <wchar.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  wcsncat
 *
 * Description:
 *    Appends the first num wide characters of source to destination, plus a
 *    terminating null wide character. If the length of the C wide string in
 *    source is less than num, only the content up to the terminating null
 *    wide character is copied.
 *
 * Input Parameters:
 *   d - the dest wchar string that contains the concatenated resulting
 *       string
 *   s - the source wchar string to be appended
 *   n - maximum number of characters to be appended
 *
 * Returned Value:
 *   The pointer point to the begin of the concatenated string
 *
 ****************************************************************************/

FAR wchar_t *wcsncat(FAR wchar_t *d, FAR const wchar_t *s, size_t n)
{
  FAR wchar_t *a = d;

  d += wcslen(d);
  while (n-- && *s)
    {
      *d++ = *s++;
    }

  *d++ = 0;
  return a;
}
