/****************************************************************************
 * graphics/nxglib/pwfb/pwfb_filltrapezoid.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <fixedmath.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxbe.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Make sure that this file is used in the proper context */

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_filltrapezoid*bpp
 *
 * Description:
 *   Fill a trapezoidal region in the framebuffer memory with a fixed color.
 *   Clip the trapezoid to lie within a boundng box.  This is useful for
 *   drawing complex shapes that can be broken into a set of trapezoids.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(pwfb_filltrapezoid, NXGLIB_SUFFIX)
(
  FAR struct nxbe_window_s *bwnd,
  FAR const struct nxgl_trapezoid_s *trap,
  FAR const struct nxgl_rect_s *bounds,
  NXGL_PIXEL_T color)
{
  unsigned int stride;
  unsigned int width;
  FAR uint8_t *dest;
  FAR uint8_t *line;
  int nrows;
  b16_t x1;
  b16_t x2;
  nxgl_coord_t y1;
  nxgl_coord_t y2;
  b16_t dx1dy;
  b16_t dx2dy;

#if NXGLIB_BITSPERPIXEL < 8
  uint8_t mpixel = NXGL_MULTIPIXEL(color);
  uint8_t mask;
  int lnlen;
#endif

  /* Get the width of the framebuffer in bytes */

  stride = bwnd->stride;

  /* Get the top run position and the number of rows to draw */

  x1    = trap->top.x1;
  x2    = trap->top.x2;

  /* Calculate the number of rows to render */

  y1    = trap->top.y;
  y2    = trap->bot.y;
  nrows = y2 - y1 + 1;

  /* Calculate the slope of the left and right side of the trapezoid */

  if (nrows > 1)
    {
      dx1dy = b16divi((trap->bot.x1 - x1), nrows - 1);
      dx2dy = b16divi((trap->bot.x2 - x2), nrows - 1);
    }
  else
    {
      /* The trapezoid is a run! Use the average width. */

      x1    = (x1 + trap->bot.x1) >> 1;
      x2    = (x2 + trap->bot.x2) >> 1;
      dx1dy = 0;
      dx2dy = 0;
    }

  /* Perform vertical clipping */

  if (y1 < bounds->pt1.y)
    {
      /* Is the entire trapezoid "above" the clipping window? */

      if (y2 < bounds->pt1.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Calculate the x values for the new top run */

      int dy = bounds->pt1.y - y1;
      x1    += dy * dx1dy;
      x2    += dy * dx2dy;

      /* Clip and re-calculate the number of rows to render */

      y1     = bounds->pt1.y;
      nrows  = y2 - y1 + 1;
    }

  if (y2 > bounds->pt2.y)
    {
      /* Is the entire trapezoid "below" the clipping window? */

      if (y1 > bounds->pt2.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Clip and re-calculate the number of rows to render */

      y2     = bounds->pt2.y;
      nrows  = y2 - y1 + 1;
    }

  /* Get the address of the first byte on the first line */

  line = (FAR uint8_t *)bwnd->fbmem + y1 * stride ;

  /* Then fill the trapezoid line-by-line */

  while (nrows--)
    {
#ifdef CONFIG_NX_ANTIALIASING
      b16_t frac;
#endif
      int ix1;
      int ix2;

      /* Handle the special case where the sides cross (as in an hourglass) */

      if (x1 > x2)
        {
          b16_t tmp;
          ngl_swap(x1, x2, tmp);
          ngl_swap(dx1dy, dx2dy, tmp);
        }

      /* Convert the positions to integer */

      ix1 = b16toi(x1);
      ix2 = b16toi(x2);

      /* Handle some corner cases where we draw nothing.  Otherwise, we will
       * always draw at least one pixel.
       */

      if (x1 <= x2 && ix2 >= bounds->pt1.x && ix1 <= bounds->pt2.x)
        {
          /* Get a clipped copies of the starting and ending X positions.
           * This clipped truncates "down" and gives the quantized pixel
           * holding the fractional X position
           */

          ix1 = ngl_clipl(ix1, bounds->pt1.x);
          ix2 = ngl_clipr(ix2, bounds->pt2.x);

          /* Get the run length for the clipped row */

          width = ix2 - ix1 + 1;

#if NXGLIB_BITSPERPIXEL < 8
          /* Handle masking of the fractional initial byte */

#ifdef CONFIG_NX_PACKEDMSFIRST
          mask  = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(ix1)));
#else
          mask  = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(ix1)));
#endif
          dest  = line;
          lnlen = width;

          if (lnlen > 1 && mask)
            {
              dest[0] = (dest[0] & ~mask) | (mpixel & mask);
              mask = 0xff;
              dest++;
              lnlen--;
            }

          /* Handle masking of the fractional final byte */

#ifdef CONFIG_NX_PACKEDMSFIRST
          mask &= (uint8_t)(0xff << (8 - NXGL_REMAINDERX(ix2)));
#else
          mask &= (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(ix2)));
#endif
          if (lnlen > 0 && mask)
            {
              dest[lnlen - 1] = (dest[lnlen - 1] & ~mask) | (mpixel & mask);
              lnlen--;
            }

          /* Handle all of the unmasked bytes in-between */

          if (lnlen > 0)
            {
              NXGL_MEMSET(dest, (NXGL_PIXEL_T)color, lnlen);
            }

#else /* NXGLIB_BITSPERPIXEL < 8 */

          /* Then draw the run from (line + ix1) to (line + ix2) */

          dest = line + NXGL_SCALEX(ix1);

#ifdef CONFIG_NX_ANTIALIASING

          /* Perform blending on the first pixel of the row */

          frac = b16ONE - b16frac(x1);
          NXGL_BLEND(dest, (NXGL_PIXEL_T)color, frac);
          dest += NXGL_SCALEX(1);
          width--;

          if (width > 0)
            {
              /* Copy pixels between the first and last pixel of the row. */

              if (width > 1)
                {
                  NXGL_MEMSET(dest, (NXGL_PIXEL_T)color, width - 1);
                }

              /* And blend the final pixel */

              dest += NXGL_SCALEX(width - 1);
              frac  = b16frac(x2);
              NXGL_BLEND(dest, (NXGL_PIXEL_T)color, frac);
            }

#else /* CONFIG_NX_ANTIALIASING */

          NXGL_MEMSET(dest, (NXGL_PIXEL_T)color, width);

#endif /* CONFIG_NX_ANTIALIASING */
#endif /* NXGLIB_BITSPERPIXEL < 8 */
        }

      /* Move to the start of the next line */

      line += stride;

      /* Add the dx/dy value to get the run positions on the next row */

      x1   += dx1dy;
      x2   += dx2dy;
    }
}
