/****************************************************************************
 * libs/libnx/nxtk/nxtk_subwindowmove.c
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
 * Name: nxtk_subwindowmove
 *
 * Description:
 *   Perform common clipping operations in preparatons for calling nx_move()
 *
 * Input Parameters:
 *   fwnd       - The framed window within which the move is to be done.
 *                This must have been previously created by
 *                nxtk_openwindow().
 *   destrect   - The loccation to receive the clipped rectangle relative
 *                to containing window
 *   destoffset - The location to received the clipped offset.
 *   srcrect    - Describes the rectangular region relative to the client
 *                sub-window to move relative to the sub-window
 *   srcoffset  - The offset to move the region
 *   bounds     - The subwindow bounds in absolute screen coordinates.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

void nxtk_subwindowmove(FAR struct nxtk_framedwindow_s *fwnd,
                        FAR struct nxgl_rect_s *destrect,
                        FAR struct nxgl_point_s *destoffset,
                        FAR const struct nxgl_rect_s *srcrect,
                        FAR const struct nxgl_point_s *srcoffset,
                        FAR const struct nxgl_rect_s *bounds)
{
  struct nxgl_rect_s abssrc;

  /* Temporarily, position the src rectangle in absolute screen coordinates */

  nxgl_rectoffset(&abssrc, srcrect, bounds->pt1.x, bounds->pt1.y);

  /* Clip the src rectangle to lie within the client window region */

  nxgl_rectintersect(&abssrc, &abssrc, &fwnd->fwrect);

  /* Clip the source rectangle so that destination area is within the
   * window.
   */

  destoffset->x = srcoffset->x;
  if (destoffset->x < 0)
    {
      if (abssrc.pt1.x + destoffset->x < bounds->pt1.x)
        {
           abssrc.pt1.x = bounds->pt1.x - destoffset->x;
        }
    }
  else if (abssrc.pt2.x + destoffset->x > bounds->pt2.x)
    {
       abssrc.pt2.x = bounds->pt2.x - destoffset->x;
    }

  destoffset->y = srcoffset->y;
  if (destoffset->y < 0)
    {
      if (abssrc.pt1.y + destoffset->y < bounds->pt1.y)
        {
           abssrc.pt1.y = bounds->pt1.y - destoffset->y;
        }
    }
  else if (abssrc.pt2.y + destoffset->y > bounds->pt2.y)
    {
       abssrc.pt2.y = bounds->pt2.y - destoffset->y;
    }

  /* Then move the rectangle so that is relative to the containing window,
   * not the client subwindow
   */

  nxgl_rectoffset(destrect, &abssrc, -fwnd->wnd.bounds.pt1.x,
                  -fwnd->wnd.bounds.pt1.y);
}
