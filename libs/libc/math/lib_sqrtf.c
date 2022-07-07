/****************************************************************************
 * libs/libc/math/lib_sqrtf.c
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_LIBM_ARCH_SQRTF
float sqrtf(float x)
{
  float y;

  /* Filter out invalid/trivial inputs */

  if (x < 0.0F)
    {
      set_errno(EDOM);
      return NAN_F;
    }

  if (isnan(x))
    {
      return NAN_F;
    }

  if (isinf_f(x))
    {
      return INFINITY_F;
    }

  if (x == 0.0F)
    {
      return 0.0F;
    }

  /* Guess square root (using bit manipulation) */

  y = lib_sqrtapprox(x);

  /* Perform three iterations of approximation. This number (3) is
   * definitely optimal
   */

  y = 0.5F * (y + x / y);
  y = 0.5F * (y + x / y);
  y = 0.5F * (y + x / y);

  return y;
}
#endif
