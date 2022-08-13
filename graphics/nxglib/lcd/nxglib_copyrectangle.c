/****************************************************************************
 * graphics/nxglib/lcd/nxglib_copyrectangle.c
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
#include <assert.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"
#include "nxglib_copyrun.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_copyrectangle_*bpp
 *
 * Description:
 *   Copy a rectangular bitmap image into the specific position in the
 *   framebuffer memory.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_copyrectangle, NXGLIB_SUFFIX)
(
  FAR struct lcd_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *dest,
  FAR const void *src,
  FAR const struct nxgl_point_s *origin,
  unsigned int srcstride)
{
  FAR const uint8_t *sline;
  unsigned int ncols;
  unsigned int row;
  unsigned int xoffset;
#if NXGLIB_BITSPERPIXEL < 8
  unsigned int remainder;
#endif

  /* Get the dimensions of the rectangle to fill: width in pixels,
   * height in rows
   */

  ncols = dest->pt2.x - dest->pt1.x + 1;

  /* Set up to copy the image */

  xoffset = dest->pt1.x - origin->x;
  sline = (FAR const uint8_t *)src + NXGL_SCALEX(xoffset) +
           (dest->pt1.y - origin->y) * srcstride;
#if NXGLIB_BITSPERPIXEL < 8
  remainder = NXGL_REMAINDERX(xoffset);
#endif

  /* Copy the image, one row at a time */

  for (row = dest->pt1.y; row <= dest->pt2.y; row++)
    {
#if NXGLIB_BITSPERPIXEL < 8
      /* if the source pixel is not aligned with a byte boundary, then we
       * will need to copy the image data to the run buffer first.
       */

      if (remainder != 0)
        {
          NXGL_FUNCNAME(nxgl_copyrun, NXGLIB_SUFFIX)(sline,
                                                     pinfo->buffer,
                                                     remainder,
                                                     ncols);
          pinfo->putrun(pinfo->dev, row, dest->pt1.x, pinfo->buffer, ncols);
        }
      else
#endif
        {
          /* The pixel data is byte aligned.
           * Copy the image data directly from the image memory.
           */

          pinfo->putrun(pinfo->dev, row, dest->pt1.x, sline, ncols);
        }

      /* Then adjust the source pointer to refer to the next line in
       * the source image.
       */

      sline += srcstride;
    }
}
