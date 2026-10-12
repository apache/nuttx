/****************************************************************************
 * libs/libm/libm/lib_frexpf.c
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

#include <nuttx/compiler.h>

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float frexpf(float x, FAR int *exponent)
{
  union
    {
      float f;
      uint32_t i;
    } u;

  int e;

  u.f = x;
  e = (int)(u.i >> 23) & 0xff;

  if (e == 0)
    {
      if (x == 0.0F)
        {
          *exponent = 0;
          return x;
        }

      /* Subnormal: scale into the normal range first */

      x = frexpf(x * 0x1p64f, exponent);
      *exponent -= 64;
      return x;
    }
  else if (e == 0xff)
    {
      /* Infinity or NaN */

      *exponent = 0;
      return x;
    }

  /* Keep the sign and the fraction, and set the exponent of 0.5 */

  *exponent = e - 0x7e;
  u.i &= UINT32_C(0x807fffff);
  u.i |= UINT32_C(0x3f000000);
  return u.f;
}
