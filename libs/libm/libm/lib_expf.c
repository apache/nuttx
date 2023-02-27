/****************************************************************************
 * libs/libm/libm/lib_expf.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <math.h>

#include "libm.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static float _flt_inv_fact[] =
{
  1.0 / 1.0,                    /* 1/0! */
  1.0 / 1.0,                    /* 1/1! */
  1.0 / 2.0,                    /* 1/2! */
  1.0 / 6.0,                    /* 1/3! */
  1.0 / 24.0,                   /* 1/4! */
  1.0 / 120.0,                  /* 1/5! */
  1.0 / 720.0,                  /* 1/6! */
  1.0 / 5040.0,                 /* 1/7! */
  1.0 / 40320.0,                /* 1/8! */
  1.0 / 362880.0,               /* 1/9! */
  1.0 / 3628800.0,              /* 1/10! */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float expf(float x)
{
  size_t int_part;
  bool invert;
  float value;
  float x0;
  size_t i;

  if (x == 0)
    {
      return 1;
    }
  else if (x < 0)
    {
      invert = true;
      x = -x;
    }
  else
    {
      invert = false;
    }

  /* Extract integer component */

  int_part = (size_t) x;

  /* set x to fractional component */

  x -= (float)int_part;

  /* Perform Taylor series approximation with eleven terms */

  value = 0.0F;
  x0 = 1.0F;
  for (i = 0; i < 10; i++)
    {
      value += x0 * _flt_inv_fact[i];
      x0 *= x;
    }

  /* Multiply by exp of the integer component */

  value *= lib_expif(int_part);

  if (invert)
    {
      return (1.0F / value);
    }
  else
    {
      return value;
    }
}
