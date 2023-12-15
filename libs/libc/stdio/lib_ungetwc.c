/****************************************************************************
 * libs/libc/stdio/lib_ungetwc.c
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
#include <fcntl.h>
#include <string.h>

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ungetwc_unlocked
 *
 * Description:
 *   Put a wide character back to the stream without lock the stream
 *
 * Input Parameters:
 *   wc - the wide character to be put back
 *   f - the file stream to put the wide character back to
 *
 * Returned Value:
 *   Return the wide character that put back to the stream on success,
 *   return WEOF on failed, and keep the given stream remains unchanged.
 *
 ****************************************************************************/

wint_t ungetwc_unlocked(wint_t wc, FAR FILE *f)
{
  char mbc[MB_LEN_MAX];
  int l = 1;

  /* Stream must be open for read access */

  if ((f->fs_oflags & O_RDOK) == 0)
    {
      return WEOF;
    }

  /* Try conversion early so we can fail without locking if invalid */

  if ((l = wctomb(mbc, wc)) < 0)
    {
      return WEOF;
    }

#if CONFIG_NUNGET_CHARS > 0
  if (f->fs_nungotten + l <= CONFIG_NUNGET_CHARS)
    {
      memcpy(f->fs_ungotten + f->fs_nungotten, mbc, l);
      f->fs_nungotten += l;
      return wc;
    }
  else
#endif
    {
      return WEOF;
    }
}

/****************************************************************************
 * Name: ungetwc
 *
 * Description:
 *   Put a wide character back to the stream
 *
 * Input Parameters:
 *   wc - the wide character that need to put back to the stream
 *   f - pointer to a FILE object that identifies an input stream
 *
 * Returned Value:
 *   Return the wide character that put back to the stream on success,
 *   return WEOF on failed, and keep the given stream remains unchanged.
 *
 ****************************************************************************/

wint_t ungetwc(wint_t wc, FAR FILE *f)
{
  wint_t ret;

  /* Verify that a non-NULL stream was provided and wc is not WEOF */

  if (!f || wc == WEOF)
    {
      return WEOF;
    }

  flockfile(f);
  ret = ungetwc_unlocked(wc, f);
  funlockfile(f);
  return ret;
}

#endif /* CONFIG_FILE_STREAM */
