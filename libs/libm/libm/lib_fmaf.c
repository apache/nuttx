/****************************************************************************
 * libs/libm/libm/lib_fmaf.c
 *
 * SPDX-License-Identifier: MIT
 *
 * Adapted from musl 9b2d8a1646391d5217f9a358555aebcaab5b8aaf.
 * Portable arithmetic rewritten by Szabolcs Nagy (48a619cad667, 2026-09-09).
 *
 * Copyright © 2005-2020 Rich Felker, et al.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <float.h>
#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Binary32 products are exact in binary64, but rounding the addition to
 * binary64 and then binary32 can choose the wrong side of a halfway value.
 * Recover the lost residual and round the binary64 intermediate to odd in
 * halfway/subnormal cases. A plain (float)((double)x * y + z) is not fused:
 * removing this correction changes ties and can miss underflow/inexact.
 *
 * This needs a binary64 double. Where double is narrower (or absent, as on
 * the ez80) fmaf() is not provided, instead of returning a wrongly rounded
 * result.
 */

#if DBL_MANT_DIG == 53

float fmaf(float x, float y, float z)
{
  double xy = (double)x * y;
  union
  {
    double r;
    uint64_t i;
  } u =
  {
    .r = xy + z
  };

  int exponent = (u.i >> 52) & 0x7ff;
  int halfway = (u.i & UINT64_C(0x1fffffff)) == UINT64_C(0x10000000);
  int tiny = exponent <= 0x3ff - 126 && exponent >= 0x3ff - 149;

  if (!halfway && !tiny)
    {
      return (float)u.r;
    }

  if (exponent != 0x7ff)
    {
      int sign = u.i >> 63;
      double residual = sign == (xy < z) ? xy - u.r + z : z - u.r + xy;

      if (residual != 0.0)
        {
          u.i -= sign ^ (residual < 0.0);
          u.i |= UINT64_C(1);
        }
    }

  return (float)u.r;
}

#endif /* DBL_MANT_DIG == 53 */
