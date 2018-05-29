/****************************************************************************
 * libs/libc/math/lib_truncf.c
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

float truncf(float x)
{
  union
  {
    float f;
    uint32_t i;
  } u =
  {
    x
  };

  int e = (int)(u.i >> 23 & 0xff) - 0x7f + 9;
  uint32_t m;
  volatile float __x;

  if (e >= 23 + 9)
    {
      return x;
    }

  if (e < 9)
    {
      e = 1;
    }

  m = -1u >> e;
  if ((u.i & m) == 0)
    {
      return x;
    }

  /* Force Eval */

  __x = (x + 0x1p120f);
  (void)__x;

  u.i &= ~m;
  return u.f;
}
