/****************************************************************************
 * graphics/nxglib/lcd/nxglib_getrectangle.c
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
 *   Fetch a rectangular region from LCD GRAM memory.  The source is
 *   expressed as a rectangle.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_getrectangle, NXGLIB_SUFFIX)
(
  FAR struct lcd_planeinfo_s *pinfo,
  FAR const struct nxgl_rect_s *rect,
  FAR void *dest, unsigned int deststride)
{
  FAR uint8_t *dline;
  unsigned int ncols;
  unsigned int srcrow;

  /* Get the width of the rectangle to move in pixels. */

  ncols = rect->pt2.x - rect->pt1.x + 1;

  /* dline = address of the first row pixel */

  dline = (FAR uint8_t *)dest;

  /* Copy the rectangle */

  for (srcrow = rect->pt1.y; srcrow <= rect->pt2.y; srcrow++)
    {
      pinfo->getrun(srcrow, rect->pt1.x, dline, ncols);
      dline += deststride;
    }
}
