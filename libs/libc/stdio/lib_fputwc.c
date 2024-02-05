/****************************************************************************
 * libs/libc/stdio/lib_fputwc.c
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
#include <limits.h>
#include <ctype.h>
#include <nuttx/fs/fs.h>

#include "libc.h"

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fputwc_unlocked
 *
 * Description:
 *   Write wide character to stream without lock the stream
 *
 * Input Parameters:
 *   c - the wide character to write
 *   f - the FILE object that identifies an output stream
 *
 * Returned Value:
 *   Return the wide character that written in to the stream on success,
 *   return WEOF on fail write to the stream
 *
 ****************************************************************************/

wint_t fputwc_unlocked(wchar_t c, FAR FILE *f)
{
  char mbc[MB_LEN_MAX];
  int l;

  if (isascii(c))
    {
      c = putc_unlocked(c, f);
    }
  else
    {
      l = wctomb(mbc, c);
      if (l < 0 || lib_fwrite_unlocked(mbc, l, f) < l)
        {
          c = WEOF;
        }
    }

  return c;
}

/****************************************************************************
 * Name: fputwc
 *
 * Description:
 *   Write wide character to stream
 *
 * Input Parameters:
 *   c - the wide character to write
 *   f - the FILE object that identifies an output stream
 *
 * Returned Value:
 *   Return the wide character that written in to the stream on success,
 *   return WEOF on fail write to the stream
 *
 ****************************************************************************/

wint_t fputwc(wchar_t c, FAR FILE *f)
{
  flockfile(f);
  c = fputwc_unlocked(c, f);
  funlockfile(f);
  return c;
}

#endif /* CONFIG_FILE_STREAM */
