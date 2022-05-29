/****************************************************************************
 * libs/libc/wchar/lib_mbrtowc.c
 *
 *   Copyright (c)1999 Citrus Project,
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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

#include <errno.h>
#include <wchar.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Implemented according to https://en.wikipedia.org/wiki/UTF-8 */

#define SA        0xc2u
#define SB        0xf4u

/* Upper 6 state bits are a negative integer offset to bound-check next byte
 * equivalent to: (((b) - 0x80) | ((b) + offset)) & ~0x3f
 */

#define OOB(c, b) (((((b) >> 3) - 0x10) | \
                   (((b) >> 3) + ((int32_t)(c) >> 26))) & ~7)

/* Interval [a,b). Either a must be 80 or b must be c0, lower 3 bits clear. */

#define R(a, b)   ((uint32_t)((uint32_t)((a) == 0x80 ? 0x40u - (b) : \
                                                       0u - (a)) << 23))

#define C(x)      ((x) < 2 ? -1 : (R(0x80, 0xc0) | (x)))
#define D(x)      C((x) + 16)
#define E(x)      (((x) == 0 ? R(0xa0, 0xc0) : \
                    (x) == 0xd ? R(0x80, 0xa0) : R(0x80, 0xc0)) \
                   | (R(0x80, 0xc0) >> 6) \
                   | (x))
#define F(x)      (((x) >= 5 ? 0 : \
                    (x) == 0 ? R(0x90, 0xc0) : \
                    (x) == 4 ? R(0x80, 0x90) : R(0x80, 0xc0)) \
                   | (R(0x80, 0xc0) >> 6) \
                   | (R(0x80, 0xc0) >> 12) \
                   | (x))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This definition of g_bittab refer to link:
 * https://en.wikipedia.org/wiki/UTF-8 [Codepage layout].
 */

static const uint32_t g_bittab[] =
{
                  C(0x2), C(0x3), C(0x4), C(0x5), C(0x6), C(0x7),
  C(0x8), C(0x9), C(0xa), C(0xb), C(0xc), C(0xd), C(0xe), C(0xf),
  D(0x0), D(0x1), D(0x2), D(0x3), D(0x4), D(0x5), D(0x6), D(0x7),
  D(0x8), D(0x9), D(0xa), D(0xb), D(0xc), D(0xd), D(0xe), D(0xf),
  E(0x0), E(0x1), E(0x2), E(0x3), E(0x4), E(0x5), E(0x6), E(0x7),
  E(0x8), E(0x9), E(0xa), E(0xb), E(0xc), E(0xd), E(0xe), E(0xf),
  F(0x0), F(0x1), F(0x2), F(0x3), F(0x4)
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mbrtowc
 *
 * Description:
 *   Convert a multibyte sequence to a wide character
 *
 ****************************************************************************/

size_t mbrtowc(FAR wchar_t *pwc, FAR const char *s,
               size_t n, FAR mbstate_t *ps)
{
  FAR const unsigned char *src = (FAR const void *)s;
  static mbstate_t state;
  size_t num = n;
  wchar_t dummy;
  uint32_t c;

  if (ps == NULL)
    {
      ps = &state;
    }

  c = *(FAR uint32_t *)ps;
  if (src == NULL)
    {
      if (c != 0)
        {
          goto ilseq;
        }

      return 0;
    }
  else if (pwc == NULL)
    {
      pwc = &dummy;
    }

  if (n == 0)
    {
      return -2;
    }

  if (c == 0)
    {
      if (*src < 0x80)
        {
          return !!(*pwc = *src);
        }

      if (*src - SA > SB - SA)
        {
          goto ilseq;
        }

      c = g_bittab[*src++ - SA];
      n--;
    }

  if (n != 0)
    {
      if (OOB(c, *src) != 0)
        {
          goto ilseq;
        }

loop:
      c = (c << 6) | (*src++ - 0x80);
      n--;
      if ((c >> 31) == 0)
        {
          *(FAR uint32_t *)ps = 0;
          *pwc = c;
          return num - n;
        }

      if (n != 0)
        {
          if (*src - 0x80u >= 0x40)
            {
              goto ilseq;
            }

          goto loop;
        }
    }

  *(FAR uint32_t *)ps = c;
  return -2;

ilseq:
  *(FAR uint32_t *)ps = 0;
  set_errno(EILSEQ);
  return -1;
}
