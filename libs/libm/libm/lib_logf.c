/****************************************************************************
 * libs/libm/libm/lib_logf.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012, 2017-2018 Gregory Nutt. All rights reserved.
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

#include <math.h>
#include <float.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLT_MAX_EXP_X   88.0F

/* To avoid looping forever in particular corner cases, every LOGF_MAX_ITER
 * the error criteria is relaxed by a factor LOGF_RELAX_MULTIPLIER.
 * todo: might need to adjust the double floating point version too.
 */

#define LOGF_MAX_ITER         10
#define LOGF_RELAX_MULTIPLIER 2.0F

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: logf
 ****************************************************************************/

float logf(float x)
{
  float y;
  float y_old;
  float ey;
  float epsilon;
  float relax_factor;
  int   iter;

  y       = 0.0F;
  y_old   = 1.0F;
  epsilon = FLT_EPSILON;

  iter         = 0;
  relax_factor = 1.0F;

  while (y > y_old + epsilon || y < y_old - epsilon)
    {
      y_old = y;
      ey    = expf(y);
      y    -= (ey - x) / ey;

      if (y > FLT_MAX_EXP_X)
        {
          y = FLT_MAX_EXP_X;
        }

      if (y < -FLT_MAX_EXP_X)
        {
          y = -FLT_MAX_EXP_X;
        }

      epsilon = (fabsf(y) > 1.0F) ? fabsf(y) * FLT_EPSILON : FLT_EPSILON;

      if (++iter >= LOGF_MAX_ITER)
        {
          relax_factor *= LOGF_RELAX_MULTIPLIER;
          iter = 0;
        }

      if (relax_factor > 1.0F)
        {
          epsilon *= relax_factor;
        }
    }

  if (y == FLT_MAX_EXP_X)
    {
      return INFINITY_F;
    }

  if (y == -FLT_MAX_EXP_X)
    {
      return INFINITY_F;
    }

  return y;
}
