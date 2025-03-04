/****************************************************************************
 * libs/libc/wchar/lib_wcsrchr.c
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
 * Name:  wcsrchr
 *
 * Description:
 *    Locate last occurrence of character in wide string
 *
 * Input Parameters:
 *   s - the wide string that to search on
 *   c - the character to be located
 *
 * Returned Value:
 *   A pointer to the last occurrence of "c" in "s".
 *
 ****************************************************************************/

FAR wchar_t *wcsrchr(FAR const wchar_t *s, wchar_t c)
{
  FAR const wchar_t *p;
  for (p = s + wcslen(s); p >= s && *p != c; p--);

  return p >= s ? (FAR wchar_t *)p : 0;
}
