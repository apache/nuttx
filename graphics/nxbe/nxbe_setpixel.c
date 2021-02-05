/****************************************************************************
 * graphics/nxbe/nxbe_setpixel.c
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
#include <nuttx/nx/nx.h>

#include "nxbe.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_setpixel_s
{
  struct nxbe_clipops_s cops;
  nxgl_mxpixel_t color;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipfill
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible
 *  portions of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipfill(FAR struct nxbe_clipops_s *cops,
                          FAR struct nxbe_plane_s *plane,
                          FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_setpixel_s *fillinfo = (struct nxbe_setpixel_s *)cops;

  /* Set the pixel in the graphics device. */

  plane->dev.setpixel(&plane->pinfo, &rect->pt1, fillinfo->color);

#ifdef CONFIG_NX_UPDATE
  /* Notify external logic that the display has been updated */

  nxbe_notify_rectangle(plane->driver, rect);
#endif
}

/****************************************************************************
 * Name: nxbe_setpixel
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxbe_setpixel_dev(FAR struct nxbe_window_s *wnd,
                              FAR const struct nxgl_point_s *pos,
                              nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxbe_setpixel_s info;
  struct nxgl_rect_s rect;
  int i;

  /* Offset the position by the window origin */

  nxgl_vectoradd(&rect.pt1, pos, &wnd->bounds.pt1);

  /* Make sure that the point is within the limits of the window
   * and of the background screen.
   */

  if (!nxgl_rectinside(&wnd->bounds, &rect.pt1) ||
      !nxgl_rectinside(&wnd->be->bkgd.bounds, &rect.pt1))
    {
      return;
    }

  /* Then create a bounding box and render the point if there it
   * is exposed.
   */

  rect.pt2.x = rect.pt1.x;
  rect.pt2.y = rect.pt1.y;

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      info.cops.visible  = nxbe_clipfill;
      info.cops.obscured = nxbe_clipnull;
      info.color         = color[i];

      /* Draw the point (if it is visible) */

      nxbe_clipper(wnd->above, &rect, NX_CLIPORDER_DEFAULT,
                   &info.cops, &wnd->be->plane[i]);

#ifdef CONFIG_NX_SWCURSOR
      /* Update cursor backup memory and redraw the cursor in the modified
       * window region.
       *
       * REVISIT:  This and the following logic belongs in the function
       * nxbe_clipfill().  It is here only because the struct
       * nxbe_state_s (wnd->be) is not available at that point.
       */

      nxbe_cursor_backupdraw_dev(wnd->be, &rect, i);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_setpixel
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   rect - The location to be filled
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_setpixel(FAR struct nxbe_window_s *wnd,
                   FAR const struct nxgl_point_s *pos,
                   nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  DEBUGASSERT(wnd != NULL && pos != NULL);

#ifdef CONFIG_NX_RAMBACKED
  /* If this window supports a pre-window frame buffer then shadow the full,
   * unclipped bitmap in that framebuffer.
   * REVISIT:  The logic to set a pixel in the per-window frame buffer is
   * missing
   */

  DEBUGASSERT(!NXBE_ISRAMBACKED(wnd));
#endif

  /* Don't update hidden windows */

  if (!NXBE_ISHIDDEN(wnd))
    {
      nxbe_setpixel_dev(wnd, pos, color);

#ifdef CONFIG_NX_SWCURSOR
      /* Was the software cursor visible?
       * REVISIT: Missing logic for the case where the update clobbers a
       * single pixel in the cursor image
       */

      DEBUGASSERT(!wnd->be->cursor.visible);
#endif
    }
}
