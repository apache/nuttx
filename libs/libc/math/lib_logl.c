/****************************************************************************
 * libs/libc/math/lib_logl.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012, 2017 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatibile, MIT-style license:
 *
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LDBL_MAX_EXP_X  700.0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
long double logl(long double x)
{
  long double y;
  long double y_old;
  long double ey;
  long double epsilon;

  y = 0.0;
  y_old = 1.0;
  epsilon = LDBL_EPSILON;

  while (y > y_old + epsilon || y < y_old - epsilon)
    {
      y_old = y;
      ey    = expl(y);
      y    -= (ey - x) / ey;

      if (y > LDBL_MAX_EXP_X)
        {
          y = LDBL_MAX_EXP_X;
        }

      if (y < -LDBL_MAX_EXP_X)
        {
          y = -LDBL_MAX_EXP_X;
        }

      epsilon = (fabsl(y) > 1.0) ? fabsl(y) * LDBL_EPSILON : LDBL_EPSILON;
    }

  if (y == LDBL_MAX_EXP_X)
    {
      return INFINITY;
    }

  if (y == -LDBL_MAX_EXP_X)
    {
      return INFINITY;
    }

  return y;
}
#endif
