/****************************************************************************
 * graphics/nxglib/nxglib_filltrapezoid.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <fixedmath.h>
#include <nuttx/fb.h>
#include <nuttx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp
 *
 * Descripton:
 *   Fill a trapezoidal region in the framebuffer memory with a fixed color.
 *   This is useful for drawing complex shape -- (most) complex shapes can be
 *   broken into a set of trapezoids.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxglib_filltrapezoid,NXGLIB_SUFFIX)
(FAR struct fb_videoinfo_s *vinfo, FAR struct fb_planeinfo_s *pinfo,
 FAR const struct nxgl_trapezoid_s *trap, NXGL_PIXEL_T color)
{
  unsigned int stride;
  ubyte *line;
  int nrows;
  b16_t x1;
  b16_t x2;
  b16_t dx1dy;
  b16_t dx2dy;

  /* Get the width of the framebuffer in bytes */

  stride = pinfo->stride;

  /* Get the top run position and the number of rows to draw */

  x1     = trap->top.x1;
  x2     = trap->top.x2;
  nrows  = trap->bot.y - trap->top.y + 1;

  /* Get the address of the first byte on the first line */

  line   = pinfo->fbmem + trap->top.y * stride ;

  /* Calculate the slope of the left and right side of the trapezoid */

  dx1dy  = b16divi((trap->bot.x1 - x1), nrows);
  dx2dy  = b16divi((trap->bot.x2 - x2), nrows);

  /* Then fill the trapezoid line-by-line */

  while (nrows--)
    {
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

      if (x1 > x2 || ix2 < 0 || ix1 > vinfo->yres)
        {
          /* Get a clipped copies of the startingand ending X positions.  This
           * clipped truncates "down" and gives the quantized pixel holding the
           * fractional X position
           */

          ix1 = ngl_clipl(ix1, 0);
          ix2 = ngl_clipr(ix2, vinfo->yres);

          /* Then draw the run from (line + clipx1) to (line + clipx2) */

          NXGL_MEMSET(line + NXGL_SCALEX(ix1), color, ix2 - ix1 + 1);
        }

      /* Move to the start of the next line */

      line += stride;

      /* Add the dx/dy value to get the run positions on the next row */

      x1   += dx1dy;
      x2   += dx2dy;
    }
}
