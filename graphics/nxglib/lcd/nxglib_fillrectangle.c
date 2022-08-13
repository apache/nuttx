/****************************************************************************
 * graphics/nxglib/lcd/nxglib_fillrectangle.c
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

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"
#include "nxglib_fillrun.h"

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
 * Name: nxgl_fillrectangle_*bpp
 *
 * Description:
 *   Fill a rectangle region in the LCD memory with a fixed color
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_fillrectangle, NXGLIB_SUFFIX)
(
  FAR struct lcd_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *rect,
  NXGL_PIXEL_T color)
{
  unsigned int ncols;
  unsigned int row;

  /* Get the dimensions of the rectangle to fill in pixels */

  ncols  = rect->pt2.x - rect->pt1.x + 1;

  /* Fill the run buffer with the selected color */

  NXGL_FUNCNAME(nxgl_fillrun, NXGLIB_SUFFIX)((NXGLIB_RUNTYPE *)pinfo->buffer,
                                                               color,
                                                               ncols);

  /* Then fill the rectangle line-by-line */

  for (row = rect->pt1.y; row <= rect->pt2.y; row++)
    {
      /* Draw the raster line at this row */

      pinfo->putrun(pinfo->dev, row, rect->pt1.x, pinfo->buffer, ncols);
    }
}
