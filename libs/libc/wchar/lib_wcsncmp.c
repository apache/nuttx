/****************************************************************************
 * libs/libc/wchar/lib_wcsncmp.c
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
 * Name:  wcsncmp
 *
 * Description:
 *    Compares up to "n" characters of the C wide string "l" to those of
 *    the C wide string "r".
 *
 * Input Parameters:
 *   l - the wchar string to be compared
 *   r - the wchar string to be compared
 *   n - maximum number of wide characters to be compared
 *
 * Returned Value:
 *   Return 0 if the characters compared in both strings form the same
 *   string.
 *   Return value greater than 0 if the first character that does not match
 *   has a greater value in "l" than in "r"
 *   Reutrn value less than 0 otherswise
 *
 ****************************************************************************/

int wcsncmp(FAR const wchar_t *l, FAR const wchar_t *r, size_t n)
{
  for (; n && *l == *r && *l && *r; n--, l++, r++);
  return n ? *l - *r : 0;
}
