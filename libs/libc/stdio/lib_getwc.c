/****************************************************************************
 * libs/libc/stdio/lib_getwc.c
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

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getwc
 *
 * Description:
 *   Get wide character from stream
 *
 * Input Parameters:
 *   f - Pointer to a FILE object that identifies an input stream
 *
 * Returned Value:
 *   Return the character read is returned,
 *   Return WEOF is the sequence of bytes that read cannot be interpreted as
 *   a valid wide characted, and sets the errno to EILSEQ
 *
 ****************************************************************************/

wint_t getwc(FAR FILE *f)
{
  return fgetwc(f);
}

#endif /* CONFIG_FILE_STREAM */
