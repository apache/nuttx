/****************************************************************************
 * libs/libm/libm/lib_logl.c
 *
 * SPDX-License-Identifier: ISC
 * SPDX-FileCopyrightText: Copyright (C) 2012 Gregory Nutt.
 * SPDX-FileContributor: Ported by: Darcy Gong
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

#define LDBL_MAX_EXP_X  700.0

/* Like log(): relax the error criteria by LOGL_RELAX_MULTIPLIER after every
 * LOGL_MAX_ITER iterations, to avoid looping forever.
 */

#define LOGL_MAX_ITER         400
#define LOGL_RELAX_MULTIPLIER 2.0L

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
  long double rf;
  int iter;

  y = 0.0;
  y_old = 1.0;
  epsilon = LDBL_EPSILON;

  iter = 0;
  rf = 1.0L;

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

      epsilon = ((fabsl(y) > rf) ? fabsl(y) : rf) * LDBL_EPSILON;

      if (++iter >= LOGL_MAX_ITER)
        {
          rf *= LOGL_RELAX_MULTIPLIER;
          iter = 0;
        }
    }

  if (y == LDBL_MAX_EXP_X)
    {
      return INFINITY_L;
    }

  if (y == -LDBL_MAX_EXP_X)
    {
      return INFINITY_L;
    }

  return y;
}
#endif
