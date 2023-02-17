/****************************************************************************
 * libs/libm/libm/lib_trunc.c
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
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double trunc(double x)
{
  union
  {
    double f;
    uint64_t i;
  } u =
  {
    x
  };

  int e = (int)(u.i >> 52 & 0x7ff) - 0x3ff + 12;
  uint64_t m;
  volatile float __x;

  if (e >= 52 + 12)
    {
      return x;
    }

  if (e < 12)
    {
      e = 1;
    }

  m = -1ull >> e;
  if ((u.i & m) == 0)
    {
      return x;
    }

  /* Force Evaluation */

  __x = (x + 0x1p120f);
  UNUSED(__x);

  u.i &= ~m;
  return u.f;
}
#endif
