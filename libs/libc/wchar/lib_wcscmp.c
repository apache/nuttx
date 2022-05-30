/****************************************************************************
 * libs/libc/wchar/lib_wcscmp.c
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

#include <nuttx/config.h>
#include <string.h>
#include <wchar.h>

#ifdef CONFIG_LIBC_WCHAR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcscmp
 *
 * Description:
 *   The wcscmp() function returns zero if the wide-character strings at s1
 *   and s2 are equal. It returns an integer greater than zero if at the
 *   first differing position i, the corresponding wide-character s1[i] is
 *   greater than s2[i].  It returns an integer less than zero if at the
 *   first differing position i, the corresponding wide-character s1[i] is
 *   less than s2[i]
 *
 ****************************************************************************/

int wcscmp(FAR const wchar_t *s1, FAR const wchar_t *s2)
{
  while (*s1 == *s2++)
    {
      if (*s1++ == 0)
        {
          return 0;
        }
    }

  return *s1 - *--s2;
}
#endif
