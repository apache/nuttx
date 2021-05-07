/****************************************************************************
 * libs/libnx/nxtk/nxtk_bitmapwindow.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nx.h>

#include "nxtk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtk_bitmapwindow
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified client sub-window.
 *
 * Input Parameters:
 *   hfwnd    The client sub0window that will receive the bitmap image
 *   dest   - Describes the rectangular region on in the client sub-window
 *            will receive the bit map.
 *   src    - The start of the source image.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in sub-window coordinates, however,
 *            the origin may lie outside of the sub-window display.
 *   stride - The width of the full source image in pixels.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxtk_bitmapwindow(NXTKWINDOW hfwnd, FAR const struct nxgl_rect_s *dest,
                      FAR const void **src,
                      FAR const struct nxgl_point_s *origin,
                      unsigned int stride)
{
  FAR struct nxtk_framedwindow_s *fwnd =
                                 (FAR struct nxtk_framedwindow_s *)hfwnd;
  struct nxgl_point_s wndorigin;
  struct nxgl_rect_s clipdest;

#ifdef CONFIG_DEBUG_FEATURES
  if (!hfwnd || !dest || !src || !origin)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  /* Clip the rectangle so that it lies within the sub-window bounds
   * then move the rectangle to that it is relative to the containing
   * window.
   */

  nxtk_subwindowclip(fwnd, &clipdest, dest, &fwnd->fwrect);

  /* Just return if completely outside screen */

  if (nxgl_nullrect(&clipdest))
    {
      return OK;
    }

  /* Now, move the bitmap origin so that it is relative to the containing
   * window, not the sub-window.
   *
   * Temporarily, position the origin in absolute screen coordinates
   */

  nxgl_vectoradd(&wndorigin, origin, &fwnd->fwrect.pt1);

  /* Then move the origin so that is relative to the containing window, not
   * the client subwindow
   */

  nxgl_vectsubtract(&wndorigin, &wndorigin, &fwnd->wnd.bounds.pt1);

  /* Then copy the bitmap */

  nx_bitmap((NXWINDOW)hfwnd, &clipdest, src, &wndorigin, stride);
  return OK;
}
