/****************************************************************************
 * libs/libc/stdio/lib_fgetwc.c
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
#include <errno.h>
#include <string.h>

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fgetwc_unlocked
 *
 * Description:
 *   Get wide character from stream without lock the stream
 *
 * Input Parameters:
 *   f - Pointer to a FILE object that identifies an input stream, during
 *   the read operaiton, the stream will not be locked
 *
 * Returned Value:
 *   Return the character read is returned,
 *   Return WEOF is the sequence of bytes that read cannot be interpreted as
 *   a valid wide characted, and sets the errno to EILSEQ
 *
 ****************************************************************************/

wint_t fgetwc_unlocked(FAR FILE *f)
{
  mbstate_t st;
  wchar_t wc;
  int c;
  char b;
  size_t l = -2;

  memset(&st, 0, sizeof(mbstate_t));

  /* Convert character byte-by-byte */

  while (l == -2)
    {
      b = c = getc_unlocked(f);
      if (c < 0)
        {
          if (!mbsinit(&st))
            {
              set_errno(EILSEQ);
            }

          f->fs_flags |= __FS_FLAG_EOF;
          return WEOF;
        }

      l = mbrtowc(&wc, &b, 1, &st);
      if (l == -1)
        {
          f->fs_flags |= __FS_FLAG_EOF;
          return WEOF;
        }
    }

  return wc;
}

/****************************************************************************
 * Name: fgetwc
 *
 * Description:
 *   Get wide character from stream
 *
 * Input Parameters:
 *   f - Pointer to a FILE object that identifies an input stream, the stream
 *   will be locked during the read operation
 *
 * Returned Value:
 *   Return the character read is returned,
 *   Return WEOF is the sequence of bytes that read cannot be interpreted as
 *   a valid wide characted, and sets the errno to EILSEQ
 *
 ****************************************************************************/

wint_t fgetwc(FAR FILE *f)
{
  wint_t c;
  flockfile(f);
  c = fgetwc_unlocked(f);
  funlockfile(f);
  return c;
}

#endif /* CONFIG_FILE_STREAM */
