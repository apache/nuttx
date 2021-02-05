/****************************************************************************
 * graphics/nxbe/nxbe_fill.c
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

struct nxbe_fill_s
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
  struct nxbe_fill_s *fillinfo = (struct nxbe_fill_s *)cops;

  /* Draw the rectangle to the graphics device. */

  plane->dev.fillrectangle(&plane->pinfo, rect, fillinfo->color);

#ifdef CONFIG_NX_UPDATE
  /* Notify external logic that the display has been updated */

  nxbe_notify_rectangle(plane->driver, rect);
#endif
}

/****************************************************************************
 * Name: nxbe_fill_dev
 *
 * Description:
 *  Fill the specified rectangle in the window in device memory with the
 *  specified color, performing clipping as needed.
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

static inline void nxbe_fill_dev(FAR struct nxbe_window_s *wnd,
                                 FAR const struct nxgl_rect_s *rect,
                                 nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxbe_fill_s info;
  int i;

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      DEBUGASSERT(wnd->be->plane[i].dev.fillrectangle != NULL);

      /* Fill the visible part of the rectangle */

      info.cops.visible  = nxbe_clipfill;
      info.cops.obscured = nxbe_clipnull;
      info.color         = color[i];

      nxbe_clipper(wnd->above, rect, NX_CLIPORDER_DEFAULT,
                   &info.cops, &wnd->be->plane[i]);

#ifdef CONFIG_NX_SWCURSOR
      /* Backup and redraw the cursor in the affected region.
       *
       * REVISIT:  This and the following logic belongs in the function
       * nxbe_clipfill().  It is here only because the struct nxbe_state_s
       * (wnd->be) is not available at that point.  This may result in an
       * excessive number of cursor updates.
       */

      nxbe_cursor_backupdraw_dev(wnd->be, rect, i);
#endif
    }
}

/****************************************************************************
 * Name: nxbe_fill_pwfb
 *
 * Description:
 *  Fill the specified rectangle in the per-window frame buffer with no
 *  clipping.
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

#ifdef CONFIG_NX_RAMBACKED
static inline void nxbe_fill_pwfb(FAR struct nxbe_window_s *wnd,
                                  FAR const struct nxgl_rect_s *rect,
                                  nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxgl_rect_s relrect;

  DEBUGASSERT(wnd->be->plane[0].pwfb.fillrectangle != NULL);

  /* The rectangle that we receive here is in absolute device coordinates.
   * We need to restore this to windows relative coordinates.
   */

  nxgl_rectoffset(&relrect, rect, -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  /* Copy the rectangular region to the framebuffer (no clipping).
   * REVISIT:  Assumes a single color plane.
   */

  wnd->be->plane[0].pwfb.fillrectangle(wnd, &relrect, color[0]);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_fill
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

void nxbe_fill(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxgl_rect_s remaining;

  DEBUGASSERT(wnd != NULL && rect != NULL && color != NULL);
  DEBUGASSERT(wnd->be != NULL && wnd->be->plane != NULL);

  /* Offset the rectangle by the window origin to convert it into a
   * bounding box
   */

  nxgl_rectoffset(&remaining, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Clip to the bounding box to the limits of the window and of the
   * background screen
   */

  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  if (!nxgl_nullrect(&remaining))
    {
#ifdef CONFIG_NX_RAMBACKED
      /* If this window supports a pre-window frame buffer then shadow the
       * full, unclipped bitmap in that framebuffer.
       */

      if (NXBE_ISRAMBACKED(wnd))
        {
          nxbe_fill_pwfb(wnd, &remaining, color);
        }
#endif

      /* Don't update hidden windows */

      if (!NXBE_ISHIDDEN(wnd))
        {
          /* Rend the bitmap directly to the graphics device */

          nxbe_fill_dev(wnd, &remaining, color);
        }
    }
}
