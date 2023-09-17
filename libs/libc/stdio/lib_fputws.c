/****************************************************************************
 * libs/libc/stdio/lib_fputws.c
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

#include <stdio.h>
#include <wchar.h>

#include "libc.h"

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fputws_unlocked
 *
 * Description:
 *   Write wide string to stream without lock the stream
 *
 * Input Parameters:
 *   ws - the wide string to write to the stream
 *   f - the FILE object that identifies an output stream
 *
 * Returned Value:
 *   Return the wide character that written in to the stream on success,
 *   return WEOF on fail write to the stream
 *
 ****************************************************************************/

int fputws_unlocked(FAR const wchar_t *ws, FAR FILE *f)
{
  char buf[BUFSIZ];
  size_t l = 0;

  while (ws &&
        (l = wcsrtombs(buf, &ws, sizeof(buf), NULL)) + 1 > 1)
    {
      if (lib_fwrite_unlocked(buf, l, f) < l)
        {
          funlockfile(f);
          return -1;
        }
    }

  return l;
}

/****************************************************************************
 * Name: fputws
 *
 * Description:
 *   Write wide string to stream
 *
 * Input Parameters:
 *   ws - the wide string to write to the stream
 *   f - the FILE object that identifies an output stream
 *
 * Returned Value:
 *   Return the wide character that written in to the stream on success,
 *   return WEOF on fail write to the stream
 *
 ****************************************************************************/

int fputws(FAR const wchar_t *ws, FAR FILE *f)
{
  int l;
  flockfile(f);
  l = fputws_unlocked(ws, f);
  funlockfile(f);
  return l;
}

#endif /* CONFIG_FILE_STREAM */
