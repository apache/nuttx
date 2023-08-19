/****************************************************************************
 * libs/libc/wchar/lib_wcscspn.c
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
 * Name:  wcscspn
 *
 * Description:
 *    Get span until character in wide string. scans "s" for the first
 *    occurrence of any of the wide characters that are part of "c".
 *
 * Input Parameters:
 *   s - the wchar string to be scanned
 *   c - the wchar string containing the characters to match.
 *
 * Returned Value:
 *   Return the number of wide characters of "s" read before this first
 *   occurrence.
 *
 ****************************************************************************/

size_t wcscspn(FAR const wchar_t *s, FAR const wchar_t *c)
{
  FAR const wchar_t *a;

  if (!c[0])
    {
      return wcslen(s);
    }

  if (!c[1])
    {
      return (s = wcschr(a = s, *c)) ? s - a : wcslen(a);
    }

  for (a = s; *s && !wcschr(c, *s); s++);
  return s - a;
}
