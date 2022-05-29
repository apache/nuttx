/****************************************************************************
 * libs/libc/wchar/lib_wcrtomb.c
 *
 * This code is derived from software contributed to Berkeley by
 * Chris Torek.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <wchar.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcrtomb
 *
 * Description:
 *   Convert a wide character to a multibyte sequence
 *
 ****************************************************************************/

size_t wcrtomb(FAR char *s, wchar_t wc, FAR mbstate_t *ps)
{
  if (s == NULL)
    {
      return 0;
    }
  else if ((unsigned)wc < 0x80)
    {
      *s = wc;
      return 1;
    }
  else if ((unsigned)wc < 0x800)
    {
      *s++ = 0xc0 | (wc >> 6);
      *s = 0x80 | (wc & 0x3f);
      return 2;
    }
  else if ((unsigned)wc < 0xd800 || (unsigned)wc - 0xe000 < 0x2000)
    {
      *s++ = 0xe0 | (wc >> 12);
      *s++ = 0x80 | ((wc >> 6) & 0x3f);
      *s = 0x80 | (wc & 0x3f);
      return 3;
    }
  else if ((unsigned)wc - 0x10000 < 0x100000)
    {
      *s++ = 0xf0 | (wc >> 18);
      *s++ = 0x80 | ((wc >> 12) & 0x3f);
      *s++ = 0x80 | ((wc >> 6) & 0x3f);
      *s = 0x80 | (wc & 0x3f);
      return 4;
    }

  set_errno(EILSEQ);
  return -1;
}
