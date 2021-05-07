/****************************************************************************
 * libs/libnx/nxtk/nxtk_containerclip.c
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
 * Name: nxtk_containerclip
 *
 * Description:
 *   We are given a 'src' rectangle in containing window, relative
 *   coordinates (i.e., (0,0) is the top left corner of the outer, containing
 *   window).
 *   This function will (1) clip that src rectangle so that it lies within
 *   the sub-window bounds, and then (2) move the rectangle to that it is
 *   relative to the sub-window (i.e., (0,0) is the top left corner of the
 *   sub-window).
 *
 * Input Parameters:
 *   fwnd   - The framed window to be used
 *   dest   - The locaton to put the result
 *   src    - The src rectangle in relative container-window coordinates
 *   bounds - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtk_containerclip(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *dest,
                        FAR const struct nxgl_rect_s *src,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s relbounds;

  /* The 'src' rectangle is relative to the containing window. Convert
   * the sub-window to the same origin.
   */

  nxgl_rectoffset(&relbounds, bounds, -fwnd->wnd.bounds.pt1.x,
                  -fwnd->wnd.bounds.pt1.y);

  /* The interection then leaves the portion of the containing window that
   * needs to be updated window that needs to be updated.
   */

  nxgl_rectintersect(dest, src, &relbounds);

  /* Offset this so that is relative to client subwindow origin */

  nxgl_rectoffset(dest, dest, fwnd->wnd.bounds.pt1.x - bounds->pt1.x,
                  fwnd->wnd.bounds.pt1.y - bounds->pt1.y);
}
