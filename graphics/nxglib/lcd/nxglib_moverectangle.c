/****************************************************************************
 * graphics/nxglib/lcd/nxglib_moverectangle.c
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

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxgl_moverectangle_*bpp
 *
 * Description:
 *   Move a rectangular region from location to another in the
 *   LCD memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_moverectangle, NXGLIB_SUFFIX)
(
  FAR struct lcd_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *rect,
  FAR struct nxgl_point_s *offset)
{
  unsigned int ncols;
  unsigned int srcrow;
  unsigned int destrow;

  /* Get the width of the rectangle to move in pixels. */

  ncols = rect->pt2.x - rect->pt1.x + 1;

  /* Case 1:  The destination position (offset) is above the displayed
   * position (rect)
   */

  if (offset->y < rect->pt1.y)
    {
      /* Copy the rectangle from top down */

      for (srcrow = rect->pt1.y, destrow = offset->y;
           srcrow <= rect->pt2.y;
           srcrow++, destrow++)
        {
          pinfo->getrun(pinfo->dev, srcrow, rect->pt1.x, pinfo->buffer,
                        ncols);
          pinfo->putrun(pinfo->dev, destrow, offset->x, pinfo->buffer,
                        ncols);
        }
    }

  /* Case 2: The destination position (offset) is below the displayed
   * position (rect)
   */

  else
    {
      unsigned int dy = rect->pt2.y - rect->pt1.y;

      /* Copy the rectangle from the bottom up */

      for (srcrow = rect->pt2.y, destrow = offset->y + dy;
           srcrow >= rect->pt1.y;
           srcrow--, destrow--)
        {
          pinfo->getrun(pinfo->dev, srcrow, rect->pt1.x, pinfo->buffer,
                        ncols);
          pinfo->putrun(pinfo->dev, destrow, offset->x, pinfo->buffer,
                        ncols);
        }
    }
}
