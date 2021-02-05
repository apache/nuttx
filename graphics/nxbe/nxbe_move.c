/****************************************************************************
 * graphics/nxbe/nxbe_move.c
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
#include <assert.h>

#include <nuttx/nx/nxglib.h>

#include "nxbe.h"
#include "nxmu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxbe_move_s
{
  struct nxbe_clipops_s     cops;
  struct nxgl_point_s       offset;
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s        srcrect;
  uint8_t                   order;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_clipmovesrc
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on visible
 *  regions of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmovesrc(FAR struct nxbe_clipops_s *cops,
                             FAR struct nxbe_plane_s *plane,
                             FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *info = (struct nxbe_move_s *)cops;
  struct nxgl_point_s offset;
#ifdef CONFIG_NX_UPDATE
  FAR struct nxbe_window_s *wnd;
  struct nxgl_rect_s update;
#endif

  if (info->offset.x != 0 || info->offset.y != 0)
    {
      /* Offset is the destination position of the moved rectangle */

      offset.x = rect->pt1.x + info->offset.x;
      offset.y = rect->pt1.y + info->offset.y;

      /* Move the source rectangle to the destination position in the
       * device
       */

      plane->dev.moverectangle(&plane->pinfo, rect, &offset);

#ifdef CONFIG_NX_UPDATE
      /* Move the source rectangle back to window relative coordinates and
       * apply the offset.
       */

      wnd = info->wnd;
      nxgl_rectoffset(&update, rect, offset.x - wnd->bounds.pt1.x,
                      offset.y - wnd->bounds.pt1.y);

      /* Notify any listeners that the graphic content in the update
       * rectangle has changed.
       */

      nxbe_notify_rectangle(plane->driver, &update);
#endif
    }
}

/****************************************************************************
 * Name: nxbe_clipmoveobscured
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on obsrured
 *  regions of the rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmoveobscured(FAR struct nxbe_clipops_s *cops,
                                  FAR struct nxbe_plane_s *plane,
                                  FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *info = (struct nxbe_move_s *)cops;
  struct nxgl_rect_s dst;

  nxgl_rectoffset(&dst, rect, info->offset.x, info->offset.y);
  nxmu_redraw(info->wnd, &dst);
}

/****************************************************************************
 * Name: nxbe_clipmovedest
 *
 * Description:
 *  Called from nxbe_clipper() to performed the move operation on visible
 *  regions of the source rectangle.
 *
 ****************************************************************************/

static void nxbe_clipmovedest(FAR struct nxbe_clipops_s *cops,
                              FAR struct nxbe_plane_s *plane,
                              FAR const struct nxgl_rect_s *rect)
{
  struct nxbe_move_s *dstdata = (struct nxbe_move_s *)cops;
  struct nxbe_window_s *wnd = dstdata->wnd;
  struct nxgl_point_s offset = dstdata->offset;
  struct nxgl_rect_s src;
  struct nxgl_rect_s tmprect1;
  struct nxgl_rect_s tmprect2;
  struct nxgl_rect_s nonintersecting[4];
  int i;

  /* Redraw dest regions where the source is outside of the bounds of the
   * background window
   */

  nxgl_rectoffset(&tmprect1, &dstdata->srcrect, offset.x, offset.y);
  nxgl_rectintersect(&tmprect2, &tmprect1, &wnd->be->bkgd.bounds);
  nxgl_nonintersecting(nonintersecting, rect, &tmprect2);

  for (i = 0; i < 4; i++)
    {
      if (!nxgl_nullrect(&nonintersecting[i]))
        {
          nxmu_redraw(dstdata->wnd, &nonintersecting[i]);
        }
    }

  /* Clip to determine what is inside the bounds */

  nxgl_rectintersect(&src, rect, &dstdata->srcrect);

  if (!nxgl_nullrect(&src))
    {
      struct nxbe_move_s srcinfo;

      /* Move the visible part of window */

      srcinfo.cops.visible  = nxbe_clipmovesrc;
      srcinfo.cops.obscured = nxbe_clipmoveobscured;
      srcinfo.offset        = offset;
      srcinfo.wnd           = wnd;

      nxbe_clipper(dstdata->wnd->above, &src, dstdata->order,
                   &srcinfo.cops, plane);
    }
}

/****************************************************************************
 * Name: nxbe_move_dev
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move (absolute device
 *            positions)
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nxbe_move_dev(FAR struct nxbe_window_s *wnd,
                                 FAR const struct nxgl_rect_s *rect,
                                 FAR const struct nxgl_point_s *offset)
{
  struct nxbe_move_s info;
#ifdef CONFIG_NX_SWCURSOR
  struct nxgl_rect_s dest;
#endif
  int i;

  info.cops.visible  = nxbe_clipmovedest;
  info.cops.obscured = nxbe_clipnull;
  info.offset.x      = offset->x;
  info.offset.y      = offset->y;
  info.wnd           = wnd;

  nxgl_rectcopy(&info.srcrect, rect);

  /* The clip order depends up the direction that the rectangle is being
   * moved.
   */

  if (offset->y < 0)
    {
      /* Moving rectangle up */

      if (offset->x < 0)
        {
          /* Moving to upper-left */

          info.order = NX_CLIPORDER_TLRB; /* Top-left-right-bottom */
        }
      else
        {
          /* Moving to upper-right (or just up) */

          info.order = NX_CLIPORDER_TRLB;  /* Top-right-left-bottom */
        }
    }
  else
    {
      /* Moving rectangle down (or just left/right) */

      if (offset->x < 0)
        {
          /* Moving to lower-left */

          info.order = NX_CLIPORDER_BLRT; /* Bottom-left-right-top */
        }
      else
        {
          /* Moving to lower-right */

          info.order = NX_CLIPORDER_BRLT; /* Bottom-right-left-top */
        }
    }

#ifdef CONFIG_NX_SWCURSOR
  /* Apply the offsets to the source window to get the destination window */

  nxgl_rectoffset(&dest, rect, offset->x, offset->y);
#endif

  /* Then perform the move */

#if CONFIG_NX_NPLANES > 1
  for (i = 0; i < wnd->be->vinfo.nplanes; i++)
#else
  i = 0;
#endif
    {
#ifdef CONFIG_NX_SWCURSOR
      /* Is the cursor visible? */

      if (wnd->be->cursor.visible)
        {
          /* Remove the cursor from the source region */

          wnd->be->plane[i].cursor.erase(wnd->be, rect, i);
        }
#endif

      nxbe_clipper(wnd->above, &info.srcrect, info.order,
                   &info.cops, &wnd->be->plane[i]);

#ifdef CONFIG_NX_SWCURSOR
      /* Backup and redraw the cursor in the modified region.
       *
       * REVISIT:  This and the following logic belongs in the function
       * nxbe_clipfill().  It is here only because the struct nxbe_state_s
       * (wnd->be) is not available at that point.  This may result in an
       * excessive number of cursor updates.
       */

      nxbe_cursor_backupdraw_dev(wnd->be, &dest, i);
#endif
    }
}

/****************************************************************************
 * Name: nxbe_move_pwfb
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move (absolute positions)
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NX_RAMBACKED
static inline void nxbe_move_pwfb(FAR struct nxbe_window_s *wnd,
                                  FAR const struct nxgl_rect_s *rect,
                                  FAR const struct nxgl_point_s *offset)
{
  FAR const void *src[CONFIG_NX_NPLANES];
  struct nxgl_point_s destpos;
  struct nxgl_point_s origin;
  struct nxgl_rect_s srcrect;
  struct nxgl_rect_s destrect;
  unsigned int bpp;

  /* The rectangle that we receive here is in absolute device coordinates.
   * We need to restore this to windows relative coordinates.
   */

  nxgl_rectoffset(&srcrect, rect, -wnd->bounds.pt1.x, -wnd->bounds.pt1.y);

  /* Offset is the destination position of the moved rectangle */

  destpos.x = srcrect.pt1.x + offset->x;
  destpos.y = srcrect.pt1.y + offset->y;

  /* Move the source rectangle to the destination position in the
   * frambebuffer.
   * REVISIT:  Assumes a single color plane.
   */

  DEBUGASSERT(wnd->be->plane[0].pwfb.moverectangle != NULL);
  wnd->be->plane[0].pwfb.moverectangle(wnd, &srcrect, &destpos);

  /* Construct the destination bounding box in relative window
   * coordinates.  This derives from the source bounding box with
   * an offset distination.
   */

  nxgl_rectoffset(&destrect, &srcrect, offset->x, offset->y);

  /* Get the source of address of the moved rectangle in the framebuffer. */

  bpp    = wnd->be->plane[0].pinfo.bpp;
  src[0] = (FAR const void *)
           ((FAR uint8_t *)wnd->fbmem +
            destrect.pt1.y * wnd->stride +
            ((bpp * destrect.pt1.x) >> 3));

  /* For resolutions less than 8-bits, the starting pixel will be contained
   * in the byte pointed to by src[0]but may not be properly aligned for
   * the transfer.  We fix this by modifying the origin.
   */

  origin.x = destrect.pt1.x;
  origin.y = destrect.pt1.y;

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

  /* Update the physical device by just copying the rectangle from the
   * framebuffer to the destination rectangle device graphics memory.
   */

  nxbe_flush(wnd, &destrect, src, &origin, wnd->stride);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   wnd    - The window within which the move is to be done
 *   rect   - Describes the rectangular region to move (window relative)
 *   offset - The offset to move the region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxbe_move(FAR struct nxbe_window_s *wnd,
               FAR const struct nxgl_rect_s *rect,
               FAR const struct nxgl_point_s *offset)
{
  struct nxgl_rect_s srcrect;

  DEBUGASSERT(wnd != NULL && rect != NULL && offset != 0);
  if (offset->x != 0 || offset->y != 0)
    {
      /* Offset the rectangle by the window origin to create a bounding box */

      nxgl_rectoffset(&srcrect, rect, wnd->bounds.pt1.x, wnd->bounds.pt1.y);

      /* Clip to the limits of the window and of the background screen */

      nxgl_rectintersect(&srcrect, &srcrect, &wnd->bounds);
      nxgl_rectintersect(&srcrect, &srcrect, &wnd->be->bkgd.bounds);

      if (!nxgl_nullrect(&srcrect))
        {
#ifdef CONFIG_NX_RAMBACKED
          /* Update the pre-window framebuffer first, then the device
           * memory.
           */

          if (NXBE_ISRAMBACKED(wnd))
            {
              nxbe_move_pwfb(wnd, &srcrect, offset);
            }
          else
#endif
          /* Don't update hidden windows */

          if (!NXBE_ISHIDDEN(wnd))
            {
              /* Update only the graphics device memory. */

              nxbe_move_dev(wnd, &srcrect, offset);
            }
        }
    }
}
