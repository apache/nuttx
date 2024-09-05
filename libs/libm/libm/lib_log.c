/****************************************************************************
 * libs/libm/libm/lib_log.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012, 2018 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatible, MIT-style license:
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

#define DBL_MAX_EXP_X   700.0

/* To avoid looping forever in particular corner cases, every LOG_MAX_ITER
 * the error criteria is relaxed by a factor LOG_RELAX_MULTIPLIER.
 * todo: might need to adjust the double floating point version too.
 */

#define LOG_MAX_ITER         10
#define LOG_RELAX_MULTIPLIER 2.0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: log
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE
double log(double x)
{
  double y;
  double y_old;
  double ey;
  double epsilon;
  double relax_factor;
  int    iter;

  y = 0.0;
  y_old = 1.0;
  epsilon = DBL_EPSILON;

  iter         = 0;
  relax_factor = 1.0;

  while (y > y_old + epsilon || y < y_old - epsilon)
    {
      y_old = y;
      ey    = exp(y);
      y    -= (ey - x) / ey;

      if (y > DBL_MAX_EXP_X)
        {
          y = DBL_MAX_EXP_X;
        }

      if (y < -DBL_MAX_EXP_X)
        {
          y = -DBL_MAX_EXP_X;
        }

      epsilon = (fabs(y) > 1.0) ? fabs(y) * DBL_EPSILON : DBL_EPSILON;

      if (++iter >= LOG_MAX_ITER)
        {
          relax_factor *= LOG_RELAX_MULTIPLIER;
          iter = 0;
        }

      if (relax_factor > 1.0)
        {
          epsilon *= relax_factor;
        }
    }

  if (y == DBL_MAX_EXP_X)
    {
      return INFINITY;
    }

  if (y == -DBL_MAX_EXP_X)
    {
      return INFINITY;
    }

  return y;
}
#endif
