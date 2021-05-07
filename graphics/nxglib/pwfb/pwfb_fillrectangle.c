/****************************************************************************
 * graphics/nxglib/pwfb/pwfb_fillrectangle.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwfb_fillrectangle_*bpp
 *
 * Description:
 *   Fill a rectangle region in the framebuffer memory with a fixed color
 *
 ****************************************************************************/

void NXGL_FUNCNAME(pwfb_fillrectangle, NXGLIB_SUFFIX)
(
  FAR struct nxbe_window_s *bwnd,
  FAR const struct nxgl_rect_s *rect,
  NXGL_PIXEL_T color)
{
  FAR uint8_t *line;
  unsigned int width;
  unsigned int stride;
  int rows;

#if NXGLIB_BITSPERPIXEL < 8
  FAR uint8_t *dest;
  uint8_t mpixel = NXGL_MULTIPIXEL(color);
  uint8_t leadmask;
  uint8_t tailmask;
  uint8_t mask;
  int lnlen;
#endif

  /* Get the width of the framebuffer in bytes */

  stride = bwnd->stride;

  /* Get the dimensions of the rectangle to fill in pixels */

  width  = rect->pt2.x - rect->pt1.x + 1;
  rows   = rect->pt2.y - rect->pt1.y + 1;

  /* Get the address of the first byte in the first line to write */

  line   = (FAR uint8_t *)bwnd->fbmem + rect->pt1.y * stride +
           NXGL_SCALEX(rect->pt1.x);

#if NXGLIB_BITSPERPIXEL < 8
# ifdef CONFIG_NX_PACKEDMSFIRST

  /* Get the mask for pixels that are ordered so that they pack from the
   * MS byte down.
   */

  leadmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt2.x - 1)));
# else
  /* Get the mask for pixels that are ordered so that they pack from the
   * LS byte up.
   */

  leadmask = (uint8_t)(0xff << (8 - NXGL_REMAINDERX(rect->pt1.x)));
  tailmask = (uint8_t)(0xff >> (8 - NXGL_REMAINDERX(rect->pt1.x - 1)));
# endif
#endif

  /* Then fill the rectangle line-by-line */

  while (rows-- > 0)
    {
#if NXGLIB_BITSPERPIXEL < 8
      /* Handle masking of the fractional initial byte */

      mask  = leadmask;
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

      mask &= tailmask;
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
#else
      /* Draw the entire raster line */

      NXGL_MEMSET(line, (NXGL_PIXEL_T)color, width);
#endif
      line += stride;
    }
}
