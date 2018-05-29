/****************************************************************************
 * libs/libc/math/lib_logf.c
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

#include <math.h>
#include <float.h>

#define FLT_MAX_EXP_X   88.0F

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

  y       = 0.0F;
  y_old   = 1.0F;
  epsilon = FLT_EPSILON;

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
    }

  if (y == FLT_MAX_EXP_X)
    {
      return INFINITY;
    }

  if (y == -FLT_MAX_EXP_X)
    {
      return INFINITY;
    }

  return y;
}
