/****************************************************************************
 * libs/libm/libm/lib_frexpl.c
 *
 * SPDX-License-Identifier: ISC
 * SPDX-FileCopyrightText: Copyright (C) 2012 Gregory Nutt.
 * SPDX-FileContributor: Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatible, MIT-style license:
 *
 * Copyright (C) 2009-2011 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
long double frexpl(long double x, int *exponent)
{
  long double a = x < 0.0L ? -x : x;
  int e = 0;

  /* Zero, NaN and infinity */

  if (a == 0.0L || a != a || a - a != 0.0L)
    {
      *exponent = 0;
      return x;
    }

  /* Multiplication by a power of two is exact here, for any format of
   * long double: scale by 2^64 first, then by 2.
   */

  while (a >= 0x1p64l)
    {
      a /= 0x1p64l;
      e += 64;
    }

  while (a * 0x1p64l < 1.0L)
    {
      a *= 0x1p64l;
      e -= 64;
    }

  while (a >= 1.0L)
    {
      a *= 0.5L;
      e++;
    }

  while (a < 0.5L)
    {
      a *= 2.0L;
      e--;
    }

  *exponent = e;
  return x < 0.0L ? -a : a;
}
#endif
