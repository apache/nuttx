/****************************************************************************
 * libs/libc/math/lib_asinl.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012, 2016 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_HAVE_LONG_DOUBLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static long double asinl_aux(long double x)
{
  long double y;
  long double y_cos;
  long double y_sin;

  y = 0.0;
  y_sin = 0.0;

  while (fabsl(y_sin - x) > DBL_EPSILON)
    {
      y_cos = cosl(y);
      y -= (y_sin - x) / y_cos;
      y_sin = sinl(y);
    }

  return y;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

long double asinl(long double x)
{
  long double y;

  /* Verify that the input value is in the domain of the function */

  if (x < -1.0 || x > 1.0 || isnan(x))
    {
      return NAN;
    }

  /* if x is > sqrt(2), use identity for faster convergence */

  if (fabsl(x) > 0.71)
    {
      y = M_PI_2 - asinl_aux(sqrtl(1.0 - x * x));
      y = copysignl(y, x);
    }
  else
    {
      y = asinl_aux(x);
    }

  return y;
}

#endif /* CONFIG_HAVE_LONG_DOUBLE */
