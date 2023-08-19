/****************************************************************************
 * libs/libc/wchar/lib_wcslcat.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 1998 Todd C. Miller <Todd.Miller@courtesan.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * from OpenBSD: strlcat.c,v 1.3 2000/11/24 11:10:02 itojun Exp
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
 * Name:  wcslcat
 *
 * Description:
 *     Appends src to string dst of size siz (unlike wcsncat, siz is the
 *     full size of dst, not space left).  At most siz-1 characters
 *     will be copied.  Always NUL terminates (unless siz == 0).
 *     Returns wcslen(initial dst) + wcslen(src); if retval >= siz,
 *     truncation occurred.
 *
 * Input Parameters:
 *   dst - the dest wchar string that contains the concatenated string
 *   src - the source wchar string that we need to concatenate
 *   siz - the max length that can be written to the end of the dst string
 *
 * Returned Value:
 *   The total length of the wchar string that concatenated
 *
 ****************************************************************************/

size_t wcslcat(FAR wchar_t *dst, FAR const wchar_t *src, size_t siz)
{
  FAR wchar_t *d = dst;
  FAR const wchar_t *s = src;
  size_t n = siz;
  size_t dlen;

  /* Find the end of dst and adjust bytes left but don't go past end */

  while (n-- != 0 && *d != '\0')
    {
      d++;
    }

  dlen = d - dst;
  n = siz - dlen;

  if (n == 0)
    {
      return(dlen + wcslen(s));
    }

  while (*s != '\0')
    {
      if (n != 1)
        {
          *d++ = *s;
          n--;
        }

      s++;
    }

  *d = '\0';

  /* count does not include NUL */

  return(dlen + (s - src));
}
