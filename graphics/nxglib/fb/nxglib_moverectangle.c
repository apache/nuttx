/****************************************************************************
 * graphics/nxglib/fb/nxglib_moverectangle.c
 *
 *   Copyright (C) 2008-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>

#include <nuttx/video/fb.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_lowresmemcpy
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL < 8
static inline void nxgl_lowresmemcpy(FAR uint8_t *dline, FAR const uint8_t *sline,
                                     unsigned int width,
                                     uint8_t leadmask, uint8_t tailmask)
{
  FAR const uint8_t *sptr;
  FAR uint8_t *dptr;
  uint8_t mask;
  int lnlen;

  /* Handle masking of the fractional initial byte */

  mask  = leadmask;
  sptr  = sline;
  dptr  = dline;
  lnlen = width;

  if (lnlen > 1 && mask)
     {
       dptr[0] = (dptr[0] & ~mask) | (sptr[0] & mask);
       mask = 0xff;
       dptr++;
       sptr++;
       lnlen--;
     }

   /* Handle masking of the fractional final byte */

   mask &= tailmask;
   if (lnlen > 0 && mask)
     {
       dptr[lnlen-1] = (dptr[lnlen-1] & ~mask) | (sptr[lnlen-1] & mask);
       lnlen--;
     }

   /* Handle all of the unmasked bytes in-between */

   if (lnlen > 0)
     {
       NXGL_MEMCPY(dptr, sptr, lnlen);
     }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_moverectangle_*bpp
 *
 * Description:
 *   Move a rectangular region from location to another in the
 *   framebuffer memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_moverectangle,NXGLIB_SUFFIX)
(FAR struct fb_planeinfo_s *pinfo, FAR const struct nxgl_rect_s *rect,
 FAR struct nxgl_point_s *offset)
{
  FAR const uint8_t *sline;
  FAR uint8_t *dline;
  unsigned int width;
  unsigned int stride;
  unsigned int rows;

#if NXGLIB_BITSPERPIXEL < 8
  uint8_t leadmask;
  uint8_t tailmask;
#endif

  /* Get the width of the framebuffer in bytes */

  stride = pinfo->stride;

  /* Get the dimensions of the rectangle to fill: width in pixels, height
   * in rows
   */

  width = rect->pt2.x - rect->pt1.x + 1;
  rows  = rect->pt2.y - rect->pt1.y + 1;

#if NXGLIB_BITSPERPIXEL < 8
# ifdef CONFIG_NX_PACKEDMSFIRST

  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt2.x-1)));
# else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x-1)));
# endif
#endif

  /* sline = address of the first pixel in the top row of the source in
   * framebuffer memory
   */

  sline = pinfo->fbmem + rect->pt1.y * stride + NXGL_SCALEX(rect->pt1.x);

  /* dline = address of the first pixel in the top row of the destination
   * in framebuffer memory.  We get dline by subtract the offset from the
   * source position.
   */

  dline = pinfo->fbmem + offset->y * stride + NXGL_SCALEX(offset->x);

  /* Case 1:  Is the destination position above the displayed position?
   * If the destination position is less then then the src address, then the
   * destination is offset to a position below (and or to the left) of the
   * source in framebuffer memory.
   */

  if (offset->y < rect->pt1.y ||
     (offset->y < rect->pt1.y && offset->x <= rect->pt1.x))
    {
      /* Yes.. Copy the rectangle from top down (i.e., adding the stride
       * to move to the next, lower row) */

      while (rows--)
        {
          /* Copy the row */

#if NXGLIB_BITSPERPIXEL < 8
          nxgl_lowresmemcpy(dline, sline, width, leadmask, tailmask);
#else
          NXGL_MEMCPY(dline, sline, width);
#endif
          /* Point to the next source/dest row below the current one */

          dline += stride;
          sline += stride;
        }
    }

  /* Case 2: No.. the destination position is above (or to the left of)
   * the displayed source position
   */

  else
    {
      /* Adjust sline and dline to point to the bottom row (+1) of the
       * source and destination rectangles in framebuffer memory.
       */

      unsigned int hoffset = rows * stride;
      sline += hoffset;
      dline += hoffset;

      /* Copy the rectangle from the bottom up (i.e., subtracting stride
       * to re-position to the previous, higher row)
       */

      while (rows--)
        {
          /* Point to the next source/dest row above the current one */

          dline -= stride;
          sline -= stride;

          /* Copy the row */

#if NXGLIB_BITSPERPIXEL < 8
          nxgl_lowresmemcpy(dline, sline, width, leadmask, tailmask);
#else
          NXGL_MEMCPY(dline, sline, width);
#endif
        }
    }
}
