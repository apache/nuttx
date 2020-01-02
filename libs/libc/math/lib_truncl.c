/****************************************************************************
 * libs/libc/math/lib_truncl.c
 *
 * This implementation is derived from the musl library under the MIT License
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <float.h>
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
static const long double toint = 1 / LDBL_EPSILON;

/* FIXME This will only work if long double is 64 bit and little endian */

union ldshape
{
  long double f;
  struct
    {
      uint64_t m;
      uint16_t se;
    } i;
};

long double truncl(long double x)
{
  union ldshape u =
  {
    x
  };
  int e = u.i.se & 0x7fff;
  int s = u.i.se >> 15;
  long double y;
  volatile long double __x;

  if (e >= 0x3fff + LDBL_MANT_DIG - 1)
    {
      return x;
    }

  if (e <= 0x3fff - 1)
    {
      /* Force Eval */

      __x = (x + 0x1p120f);
      UNUSED(__x);
      return x*0;
    }

  /* y = int(|x|) - |x|, where int(|x|) is an integer neighbor of |x| */

  if (s)
    {
      x = -x;
    }

  y = x + toint - toint - x;
  if (y > 0)
    {
      y -= 1;
    }

  x += y;
  return s ? -x : x;
}
#endif
