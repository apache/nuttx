/****************************************************************************
 * graphics/nxbe/nxbe_setposition.c
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

#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setposition
 *
 * Description:
 *   This function checks for intersections and redraws the display after
 *   a change in the position of a window.
 *
 ****************************************************************************/

void nxbe_setposition(FAR struct nxbe_window_s *wnd,
                      FAR const struct nxgl_point_s *pos)
{
  struct nxgl_rect_s before;
  struct nxgl_rect_s rect;

  DEBUGASSERT(wnd != NULL && pos != NULL);

  /* Back out the old window origin position from the bounding box */

  nxgl_rectoffset(&rect,
                  &wnd->bounds,
                  -wnd->bounds.pt1.x,
                  -wnd->bounds.pt1.y);

  /* Add the new window origin into the bounding box */

  nxgl_rectcopy(&before, &wnd->bounds);
  nxgl_rectoffset(&wnd->bounds, &rect, pos->x, pos->y);

  /* Get the union of the 'before' bounding box and the 'after' bounding
   * this union is the region of the display that must be updated.
   */

  nxgl_rectunion(&rect, &before, &wnd->bounds);
  nxgl_rectintersect(&rect, &rect, &wnd->be->bkgd.bounds);

  /* Report the new size/position */

  nxmu_reportposition(wnd);

  /* Then redraw this window AND all windows below it. Having moved the
   * window, we may have exposed previously obscured portions of windows
   * below this one.
   */

  nxbe_redrawbelow(wnd->be, wnd, &rect);
}
