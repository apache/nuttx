/****************************************************************************
 * libs/libc/math/lib_asin.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012, 2015-2016 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
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
#include <float.h>

#ifdef CONFIG_HAVE_DOUBLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef  DBL_EPSILON
#define DBL_EPSILON 1e-12

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This lib uses Newton's method to approximate asin(x).  Newton's Method
 * converges very slowly for x close to 1.  We can accelerate convergence
 * with the following identy:  asin(x)=Sign(x)*(Pi/2-asin(sqrt(1-x^2)))
 */

static double asin_aux(double x)
{
  long double y;
  double y_cos;
  double y_sin;

  y = 0.0;
  y_sin = 0.0;

  while (fabs(y_sin - x) > DBL_EPSILON)
    {
      y_cos = cos(y);
      y -= ((long double)y_sin - (long double)x) / (long double)y_cos;
      y_sin = sin(y);
    }

  return y;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

double asin(double x)
{
  double y;

  /* Verify that the input value is in the domain of the function */

  if (x < -1.0 || x > 1.0 || isnan(x))
    {
      return NAN;
    }

  /* if x is > sqrt(2), use identity for faster convergence */

  if (fabs(x) > 0.71)
    {
      y = M_PI_2 - asin_aux(sqrt(1.0 - x * x));
      y = copysign(y, x);
    }
  else
    {
      y = asin_aux(x);
    }

  return y;
}

#endif /* CONFIG_HAVE_DOUBLE */
