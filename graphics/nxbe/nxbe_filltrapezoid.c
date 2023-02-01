/****************************************************************************
 * graphics/nxbe/nxbe_filltrapezoid.c
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
#include <fixedmath.h>
#include <sys/param.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_filltrap_s
{
  struct nxbe_clipops_s cops;
  struct nxgl_trapezoid_s trap;
  nxgl_mxpixel_t color;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipfilltrapezoid
 *
 * Description:
 *  Called from nxbe_clipper() to performed the fill operation on visible
 *  portions of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipfilltrapezoid(FAR struct nxbe_clipops_s *cops,
                                   FAR struct nxbe_plane_s *plane,
                                   FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_filltrap_s *fillinfo = (struct nxbe_filltrap_s *)cops;
#ifdef CONFIG_NX_UPDATE
  struct nxgl_rect_s update;
#endif

  /* Draw the trapezond */

  plane->dev.filltrapezoid(&plane->pinfo, &fillinfo->trap, rect,
                           fillinfo->color);

#ifdef CONFIG_NX_UPDATE
  /* Notify external logic that the display has been updated */

  update.pt1.x = MIN(MAX(fillinfo->trap.top.x1, rect->pt1.x),
                     MAX(fillinfo->trap.bot.x1, rect->pt1.x));
  update.pt1.y = MAX(fillinfo->trap.top.y, rect->pt1.y);
  update.pt2.x = MAX(MIN(fillinfo->trap.top.x2, rect->pt2.x),
                     MIN(fillinfo->trap.bot.x2, rect->pt2.x));
  update.pt2.y = MIN(fillinfo->trap.bot.y, rect->pt2.y);

  nxbe_notify_rectangle(plane->driver, &update);
#endif
}

/****************************************************************************
 * Name: nxbe_filltrapezoid_dev
 *
 * Description:
 *  Fill the specified rectangle in the device graphics memory with the
 *  specified color
 *
 * Input Parameters:
 *   wnd    - The window structure reference
 *   bounds - Trapezoid bounding box (in absolute window coordinates)
 *   rect   - The location to be filled (in relative window coordinates)
 *   col    - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
nxbe_filltrapezoid_dev(FAR struct nxbe_window_s *wnd,
                       FAR const struct nxgl_rect_s *bounds,
                       FAR const struct nxgl_trapezoid_s *trap,
                       nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxbe_filltrap_s info;
  int i;

  info.cops.visible  = nxbe_clipfilltrapezoid;
  info.cops.obscured = nxbe_clipnull;

  nxgl_trapcopy(&info.trap, trap);

  /* Process each color plane */

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
      /* Rend any part of the trapezoid that is not occluded by a window
       * higher in the hierarchy.
       */

      info.color = color[i];
      nxbe_clipper(wnd->above, bounds, NX_CLIPORDER_DEFAULT,
                   &info.cops, &wnd->be->plane[i]);

#ifdef CONFIG_NX_SWCURSOR
      /* Backup and redraw the cursor in the modified region.
       *
       * REVISIT:  This and the following logic belongs in the function
       * nxbe_clipfill().  It is here only because the struct nxbe_state_s
       * (wnd->be) is not available at that point.  This may result in an
       * excessive number of cursor updates.
       */

      nxbe_cursor_backupdraw_dev(wnd->be, bounds, i);
#endif
    }
}

/****************************************************************************
 * Name: nxbe_filltrapezoid_pwfb
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   bounds - Trapezoid bounding box (in absolute window coordinates)
 *   rect - The location to be filled (in relative window coordinates)
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static inline void
nxbe_filltrapezoid_pwfb(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *bounds,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  FAR const void *src[CONFIG_NX_NPLANES];
  struct nxgl_trapezoid_s reltrap;
  struct nxgl_rect_s relbounds;
  struct nxgl_point_s origin;
  unsigned int bpp;

  /* Both the rectangle that we receive here are in absolute device
   * coordinates.  We need to restore both to windows relative coordinates.
   */

  nxgl_trapoffset(&reltrap, trap,
                  -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);
  nxgl_rectoffset(&relbounds, bounds,
                  -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  /* Copy the trapezoidal region to the framebuffer (no clipping).
   * REVISIT:  Assumes a single color plane.
   */

  DEBUGASSERT(wnd->be->plane[0].pwfb.filltrapezoid != NULL);
  wnd->be->plane[0].pwfb.filltrapezoid(wnd, &reltrap, &relbounds,
                                       color[0]);

  /* Get the source of address of the trapezoid bounding box in the
   * framebuffer.
   */

  bpp    = wnd->be->plane[0].pinfo.bpp;
  src[0] = (FAR const void *)
           ((FAR uint8_t *)wnd->fbmem +
            relbounds.pt1.y * wnd->stride +
            ((bpp * relbounds.pt1.x) >> 3));

  /* For resolutions less than 8-bits, the starting pixel will be contained
   * in the byte pointed to by src[0]but may not be properly aligned for
   * the transfer.  We fix this by modifying the origin.
   */

  origin.x = relbounds.pt1.x;
  origin.y = relbounds.pt1.y;

  switch (bpp)
    {
#ifndef CONFIG_NX_DISABLE_1BPP
      case 1:  /* 1 bit per pixel */
        {
          origin.x &= ~7;
        }
        break;
#endif

#ifndef CONFIG_NX_DISABLE_2BPP
      case 2:  /* 2 bits per pixel */
        {
          origin.x &= ~3;
        }
        break;
#endif

#ifndef CONFIG_NX_DISABLE_4BPP
      case 4:  /* 4 bits per pixel */
        {
          origin.x &= ~1;
        }
        break;
#endif

      default:
        break;
    }

  /* Copy the portion of the per-window framebuffer in the bounding box
   * to the device graphics memory.
   */

  nxbe_flush(wnd, &relbounds, src, &origin, wnd->stride);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_filltrapezoid
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   wnd  - The window structure reference
 *   clip - Clipping region (in relative window coordinates)
 *   rect - The location to be filled (in relative window coordinates)
 *   col  - The color to use in the fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_filltrapezoid(FAR struct nxbe_window_s *wnd,
                        FAR const struct nxgl_rect_s *clip,
                        FAR const struct nxgl_trapezoid_s *trap,
                        nxgl_mxpixel_t color[CONFIG_NX_NPLANES])
{
  struct nxgl_rect_s remaining;
  struct nxgl_rect_s absclip;
  struct nxgl_trapezoid_s devtrap;

  DEBUGASSERT(wnd != NULL && clip != NULL && trap != NULL);

  /* Offset the trapezoid by the window origin to position it within
   * the device graphics coordinate system.
   */

  nxgl_trapoffset(&devtrap, trap, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

  /* Create a bounding box that contains the trapezoid */

  remaining.pt1.x = b16toi(ngl_min(devtrap.top.x1, devtrap.bot.x1));
  remaining.pt1.y = devtrap.top.y;
  remaining.pt2.x = b16toi(ngl_max(devtrap.top.x2, devtrap.bot.x2));
  remaining.pt2.y = devtrap.bot.y;

  /* Clip to any user specified clipping window */

  nxgl_rectoffset(&absclip, clip, wnd->bounds.pt1.x, wnd->bounds.pt1.y);
  nxgl_rectintersect(&remaining, &remaining, &absclip);

  /* Clip to the limits of the window and of the background screen */

  nxgl_rectintersect(&remaining, &remaining, &wnd->bounds);
  nxgl_rectintersect(&remaining, &remaining, &wnd->be->bkgd.bounds);

  if (!nxgl_nullrect(&remaining))
    {
#ifdef CONFIG_NX_RAMBACKED
      /* Update the pre-window framebuffer first, then the device memory. */

      if (NXBE_ISRAMBACKED(wnd))
        {
          nxbe_filltrapezoid_pwfb(wnd, &remaining, &devtrap, color);
        }
      else
#endif
      /* Don't update hidden windows */

      if (!NXBE_ISHIDDEN(wnd))
        {
          /* Update only the graphics device memory. */

          nxbe_filltrapezoid_dev(wnd, &remaining, &devtrap, color);
        }
    }
}
