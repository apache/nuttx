/****************************************************************************
 * libs/libnx/nxtk/nxtk_subwindowclip.c
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

#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_subwindowclip
 *
 * Description:
 *   We are given a 'src' rectangle in sub-window, relative coordinates
 *   (i.e., (0,0) is the top left corner of the sub-window).  This function
 *   will (1) clip that src rectangle so that it lies within the sub-window
 *   bounds, and then (2) move the rectangle to that it is relative to the
 *   containing window (i.e., (0,0) is the top left corner of the containing
 *   window).
 *
 * Input Parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The locaton to put the result
 *   src    - The src rectangle in relative sub-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtk_subwindowclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s tmp;

  /* Temporarily, position the src rectangle in absolute screen coordinates */

  nxgl_rectoffset(&tmp, src, bounds->pt1.x, bounds->pt1.y);

  /* Clip the src rectangle to lie within the client window region */

  nxgl_rectintersect(&tmp, &tmp, bounds);

  /* Then move the rectangle so that is relative to the containing window,
   * not the client subwindow
   */

  nxgl_rectoffset(dest, &tmp,
                  -fwnd->wnd.bounds.pt1.x,
                  -fwnd->wnd.bounds.pt1.y);
}
