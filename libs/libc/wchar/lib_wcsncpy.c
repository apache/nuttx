/****************************************************************************
 * libs/libc/wchar/lib_wcsncpy.c
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
 * Name:  wcsncpy
 *
 * Description:
 *    Copies the first num characters of source to destination. If the end
 *    of the source C wide string (which is signaled by a null wide
 *    character) is found before num characters have been copied, destination
 *    is padded with additional null wide characters until a total of num
 *    characters have been written to it.
 *
 * Input Parameters:
 *   d - the destination wchar string where the content is to be copied
 *   s - the wchar string to be copied
 *   n - maximum number of wide characters to be copied from source
 *
 * Returned Value:
 *   A pointer to the destination string
 *
 ****************************************************************************/

FAR wchar_t *wcsncpy(FAR wchar_t *d, FAR const wchar_t *s, size_t n)
{
  FAR wchar_t *a = d;

  while (n && *s)
    {
      *d++ = *s++;
      n--;
    }

  wmemset(d, 0, n);
  return a;
}
