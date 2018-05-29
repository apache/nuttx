/****************************************************************************
 * libs/libc/string/lib_strxfrm.c
 *
 *   Copyright (c)1999 Citrus Project,
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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

#ifdef CONFIG_LIBC_LOCALE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strxfrm
 *
 * Description:
 *   This function transforms the string pointed to by s2 and places the
 *   resulting string into the array pointed to by s1. The transformation is
 *   such that if the strcmp() function is applied to the two transformed
 *   strings, it returns a value greater than, equal to, or less than zero,
 *   correspoinding to the result of a <<strcoll>> function applied to the
 *   same two original strings.
 *   With a C locale, this function just copies.
 *
 ****************************************************************************/

size_t strxfrm(FAR char *s1, FAR const char *s2, size_t n)
{
  size_t res;
  res = 0;
  while (n-- > 0)
    {
      if ((*s1++ = *s2++) != '\0')
        {
          ++res;
        }
      else
        {
          return res;
        }
    }
  while (*s2)
    {
      ++s2;
      ++res;
    }

  return res;
}
#endif
