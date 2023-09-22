/****************************************************************************
 * libs/libc/wchar/lib_wcspbrk.c
 *
 * Copyright © 2005-2014 Rich Felker, et al.
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
 * Name:  wcspbrk
 *
 * Description:
 *    Locate characters in wide string
 *
 * Input Parameters:
 *   s - the wide string to be scanned
 *   b - the wide string containing the characters to match
 *
 * Returned Value:
 *   The pointer to the first occurrence in "s" of any of the wide characters
 *   that are part of "b",
 *   Return null if there are no matches.
 *
 ****************************************************************************/

FAR wchar_t *wcspbrk(FAR const wchar_t *s, FAR const wchar_t *b)
{
  s += wcscspn(s, b);
  return *s ? (FAR wchar_t *)s : NULL;
}
