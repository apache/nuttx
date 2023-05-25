/****************************************************************************
 * graphics/nxglib/pwfb/pwfb_moverectangle.c
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

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxbe.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_lowresmemcpy
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL < 8
static inline void pwfb_lowresmemcpy(FAR uint8_t *dline,
                                     FAR const uint8_t *sline,
                                     unsigned int width, uint8_t leadmask,
                                     uint8_t tailmask)
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
      dptr[lnlen - 1] = (dptr[lnlen - 1] & ~mask) |
                        (sptr[lnlen - 1] & mask);
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
 * Name: pwfb_moverectangle_*bpp
 *
 * Description:
 *   Move a rectangular region from location to another in the
 *   framebuffer memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(pwfb_moverectangle, NXGLIB_SUFFIX)
(
  FAR struct nxbe_window_s *bwnd,
  FAR const struct nxgl_rect_s *rect,
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

  stride = bwnd->stride;

  /* Get the dimensions of the rectangle to fill: width in pixels, height
   * in rows
   */

  width = rect->pt2.x - rect->pt1.x + 1;
  rows  = rect->pt2.y - rect->pt1.y + 1;

#if NXGLIB_BITSPERPIXEL < 8
#  ifdef CONFIG_NX_PACKEDMSFIRST

  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt2.x - 1)));
#  else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x - 1)));
#  endif
#endif

  /* sline = address of the first pixel in the top row of the source in
   * framebuffer memory
   */

  sline = (FAR uint8_t *)bwnd->fbmem + rect->pt1.y * stride +
          NXGL_SCALEX(rect->pt1.x);

  /* dline = address of the first pixel in the top row of the destination
   * in framebuffer memory.  We get dline by subtract the offset from the
   * source position.
   */

  dline = (FAR uint8_t *)bwnd->fbmem + offset->y * stride +
          NXGL_SCALEX(offset->x);

  /* Case 1:  Is the destination position above the displayed position?
   * If the destination position is less then then the src address, then the
   * destination is offset to a position below (and or to the left) of the
   * source in framebuffer memory.
   */

  if (offset->y < rect->pt1.y ||
     (offset->y < rect->pt1.y && offset->x <= rect->pt1.x))
    {
      /* Yes.. Copy the rectangle from top down (i.e., adding the stride
       * to move to the next, lower row)
       */

      while (rows--)
        {
          /* Copy the row */

#if NXGLIB_BITSPERPIXEL < 8
          pwfb_lowresmemcpy(dline, sline, width, leadmask, tailmask);
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
          pwfb_lowresmemcpy(dline, sline, width, leadmask, tailmask);
#else
          NXGL_MEMCPY(dline, sline, width);
#endif
        }
    }
}
