/****************************************************************************
 * libs/libc/math/lib_libexpif.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong/Gregory Nutt
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

#include <sys/types.h>
#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define M_E2     (M_E   * M_E)
#define M_E4     (M_E2  * M_E2)
#define M_E8     (M_E4  * M_E4)
#define M_E16    (M_E8  * M_E8)
#define M_E32    (M_E16 * M_E16)
#define M_E64    (M_E32 * M_E32)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Values above M_E64 will never be used since it’s larger than FLT_MAX
 *(3.402823e+38).
 */

static const float g_expif_square_tbl[] =
{
  (float)M_E,    /* e^1 */
  (float)M_E2,   /* e^2 */
  (float)M_E4,   /* e^4 */
  (float)M_E8,   /* e^8 */
  (float)M_E16,  /* e^16 */
  (float)M_E32,  /* e^32 */
  (float)M_E64   /* e^64 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float lib_expif(size_t n)
{
  size_t i;
  float val;

  /* The largest calculable value for n is floor(ln(FLT_MAX)) */

  if (n > 88)
    {
      return INFINITY_F;
    }

  val = 1.0F;

  for (i = 0; n != 0; i++)
    {
      if ((n & (1 << i)) != 0)
        {
          n   &= ~(1 << i);
          val *= g_expif_square_tbl[i];
        }
    }

  return val;
}
