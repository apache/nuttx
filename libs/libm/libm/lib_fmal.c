/****************************************************************************
 * libs/libm/libm/lib_fmal.c
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

#ifdef CONFIG_HAVE_LONG_DOUBLE
#if LDBL_MANT_DIG == DBL_MANT_DIG

/****************************************************************************
 * Public Functions
 ****************************************************************************/

long double fmal(long double x, long double y, long double z)
{
  return fma(x, y, z);
}

#elif defined(__FP_FAST_FMAL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

long double fmal(long double x, long double y, long double z)
{
  return __builtin_fmal(x, y, z);
}

#else

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ld_s
{
  long double hi;
  long double lo;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* hi + lo = a + b exactly */

static struct ld_s ld_add(long double a, long double b)
{
  struct ld_s r;
  long double s;

  r.hi = a + b;
  s = r.hi - a;
  r.lo = (a - (r.hi - s)) + (b - s);
  return r;
}

/* hi + lo = a * b exactly (Dekker), for a and b far from overflow */

static struct ld_s ld_mul(long double a, long double b)
{
  const long double split =
    scalbnl(1.0L, (LDBL_MANT_DIG + 1) / 2) + 1.0L;
  struct ld_s r;
  long double ha;
  long double hb;
  long double la;
  long double lb;
  long double p;

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

static bool ld_isodd(long double x)
{
  long double m;
  int e;

  m = scalbnl(frexpl(x, &e), LDBL_MANT_DIG) * 0.5L;
  return m != truncl(m);
}

/* The next number after hi, in the direction of lo */

static long double ld_bump(long double hi, long double lo)
{
  return nextafterl(hi, lo > 0.0L ? INFINITY_L : -INFINITY_L);
}

/* Round a + b to odd: if the sum is inexact, make the last bit 1 */

static long double ld_add_odd(long double a, long double b)
{
  struct ld_s sum = ld_add(a, b);

  if (sum.lo != 0.0L && !ld_isodd(sum.hi))
    {
      sum.hi = ld_bump(sum.hi, sum.lo);
    }

  return sum.hi;
}

/* a + b scaled by 2^scale, where the result is subnormal: round once */

static long double ld_add_scaled(long double a, long double b, int scale)
{
  struct ld_s sum = ld_add(a, b);
  int lost;

  if (sum.lo != 0.0L)
    {
      /* Round to odd before scalbn() rounds again.  If scalbn() drops one
       * bit only, an odd sum is a tie for it: move it toward lo instead.
       */

      lost = (LDBL_MIN_EXP - 1) - (ilogbl(sum.hi) + scale);
      if ((lost != 1) != ld_isodd(sum.hi))
        {
          sum.hi = ld_bump(sum.hi, sum.lo);
        }
    }

  return scalbnl(sum.hi, scale);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The product is split into an exact sum hi + lo and added to z with one
 * rounding (S. Boldo, G. Melquiond, "Emulation of FMA and correctly rounded
 * sums: proved algorithms using rounding to odd", IEEE Trans. Computers,
 * 2008).  Only the rounding to nearest is supported.
 */

long double fmal(long double x, long double y, long double z)
{
  struct ld_s xy;
  struct ld_s r;
  long double xs;
  long double ys;
  long double zs;
  long double adj;
  int ex;
  int ey;
  int ez;
  int spread;

  if (x == 0.0L || y == 0.0L || isinf(x) || isinf(y) || isnan(x) ||
      isnan(y))
    {
      return x * y + z;
    }

  if (z == 0.0L)
    {
      return x * y;
    }

  if (isinf(z) || isnan(z))
    {
      return z;
    }

  xs = frexpl(x, &ex);
  ys = frexpl(y, &ey);
  zs = frexpl(z, &ez);
  spread = ex + ey - ez;

  /* x * y is too small to change z */

  if (spread < -LDBL_MANT_DIG - 2)
    {
      return z;
    }

  /* z far below x * y acts only as a sticky bit: keep its sign */

  if (spread <= LDBL_MANT_DIG * 2)
    {
      zs = scalbnl(zs, -spread);
    }
  else
    {
      zs = copysignl(LDBL_MIN, zs);
    }

  xy = ld_mul(xs, ys);
  r = ld_add(xy.hi, zs);
  spread = ex + ey;

  if (r.hi == 0.0L)
    {
      /* xy.hi and zs cancel */

      return xy.lo == 0.0L ? xy.hi + zs : scalbnl(xy.lo, spread);
    }

  adj = ld_add_odd(r.lo, xy.lo);
  if (spread + ilogbl(r.hi) >= LDBL_MIN_EXP - 1)
    {
      return scalbnl(r.hi + adj, spread);
    }

  return ld_add_scaled(r.hi, adj, spread);
}

#endif
#endif
