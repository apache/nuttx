/****************************************************************************
 * libs/libm/libm/lib_fma.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_HAVE_DOUBLE
#ifdef __FP_FAST_FMA

/****************************************************************************
 * Public Functions
 ****************************************************************************/

double fma(double x, double y, double z)
{
  return __builtin_fma(x, y, z);
}

#else

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dd_s
{
  double hi;
  double lo;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* hi + lo = a + b exactly */

static struct dd_s dd_add(double a, double b)
{
  struct dd_s r;
  double s;

  r.hi = a + b;
  s = r.hi - a;
  r.lo = (a - (r.hi - s)) + (b - s);
  return r;
}

/* hi + lo = a * b exactly (Dekker), for a and b far from overflow */

static struct dd_s dd_mul(double a, double b)
{
  const double split = 0x1p27 + 1.0;
  struct dd_s r;
  double ha;
  double hb;
  double la;
  double lb;
  double p;

  p = a * split;
  ha = a - p;
  ha += p;
  la = a - ha;

  p = b * split;
  hb = b - p;
  hb += p;
  lb = b - hb;

  r.hi = a * b;
  r.lo = ((ha * hb - r.hi) + ha * lb + la * hb) + la * lb;
  return r;
}

/* Is the last bit of the significand of x 1?  x is normal. */

static bool dd_isodd(double x)
{
  union
    {
      double f;
      uint64_t i;
    } u;

  u.f = x;
  return (u.i & 1) != 0;
}

/* The next number after hi, in the direction of lo */

static double dd_bump(double hi, double lo)
{
  return nextafter(hi, lo > 0.0 ? INFINITY : -INFINITY);
}

/* Round a + b to odd: if the sum is inexact, make the last bit 1 */

static double dd_add_odd(double a, double b)
{
  struct dd_s sum = dd_add(a, b);

  if (sum.lo != 0.0 && !dd_isodd(sum.hi))
    {
      sum.hi = dd_bump(sum.hi, sum.lo);
    }

  return sum.hi;
}

/* a + b scaled by 2^scale, where the result is subnormal: round once */

static double dd_add_scaled(double a, double b, int scale)
{
  struct dd_s sum = dd_add(a, b);
  int lost;

  if (sum.lo != 0.0)
    {
      /* Round to odd before scalbn() rounds again.  If scalbn() drops one
       * bit only, an odd sum is a tie for it: move it toward lo instead.
       */

      lost = (DBL_MIN_EXP - 1) - (ilogb(sum.hi) + scale);
      if ((lost != 1) != dd_isodd(sum.hi))
        {
          sum.hi = dd_bump(sum.hi, sum.lo);
        }
    }

  return scalbn(sum.hi, scale);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The product is split into an exact sum hi + lo and added to z with one
 * rounding (S. Boldo, G. Melquiond, "Emulation of FMA and correctly rounded
 * sums: proved algorithms using rounding to odd", IEEE Trans. Computers,
 * 2008).  Only the rounding to nearest is supported.
 */

double fma(double x, double y, double z)
{
  struct dd_s xy;
  struct dd_s r;
  double xs;
  double ys;
  double zs;
  double adj;
  int ex;
  int ey;
  int ez;
  int spread;

  if (x == 0.0 || y == 0.0 || isinf(x) || isinf(y) || isnan(x) ||
      isnan(y))
    {
      return x * y + z;
    }

  if (z == 0.0)
    {
      return x * y;
    }

  if (isinf(z) || isnan(z))
    {
      return z;
    }

  xs = frexp(x, &ex);
  ys = frexp(y, &ey);
  zs = frexp(z, &ez);
  spread = ex + ey - ez;

  /* x * y is too small to change z */

  if (spread < -DBL_MANT_DIG - 2)
    {
      return z;
    }

  /* z far below x * y acts only as a sticky bit: keep its sign */

  if (spread <= DBL_MANT_DIG * 2)
    {
      zs = scalbn(zs, -spread);
    }
  else
    {
      zs = copysign(DBL_MIN, zs);
    }

  xy = dd_mul(xs, ys);
  r = dd_add(xy.hi, zs);
  spread = ex + ey;

  if (r.hi == 0.0)
    {
      /* xy.hi and zs cancel */

      return xy.lo == 0.0 ? xy.hi + zs : scalbn(xy.lo, spread);
    }

  adj = dd_add_odd(r.lo, xy.lo);
  if (spread + ilogb(r.hi) >= DBL_MIN_EXP - 1)
    {
      return scalbn(r.hi + adj, spread);
    }

  return dd_add_scaled(r.hi, adj, spread);
}

#endif
#endif
